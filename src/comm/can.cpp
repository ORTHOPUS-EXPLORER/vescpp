#include "vescpp/comm/can.hpp"

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include "vescpp/vescpp.hpp"

using namespace std::chrono_literals;

namespace vescpp::comm
{

CAN::CAN(const std::string_view& can_port) : _port(can_port)
{
  using namespace std::placeholders;
  spdlog::debug("[{}] Opening CAN Port...", _port);
  // Loop that try to open can port
  while (!_can.open(_port, std::bind(&CAN::canRXcb, this, _1)))
  {
    spdlog::debug("[{}] Retrying to open CAN port in 1 second.", _port);
    std::this_thread::sleep_for(1s);
  }
  while (!_can._rx_running)
  {
    std::this_thread::sleep_for(1ms);
  }
  spdlog::debug("[{}] CAN Port is open !", _port);
}

void CAN::canRXcb(const can_frame& frame)
{
  auto can_id = frame.can_id & CAN_ERR_MASK;
  auto& can_data = frame.data;
  auto can_len = frame.can_dlc;
  bool handled = false;
  for (const auto& [hdlr_id, hdlr_cb] : _can_handlers)
  {
    //spdlog::debug("  CAN ID vs CAN Handler ID: {} vs {}", can_id, hdlr_id);
    if (can_id != hdlr_id || !hdlr_cb) continue;

    hdlr_cb(this, can_id, can_data, can_len);
    handled = true;
  }
  if (!handled)
  {
    bool handleable = false;
    for (const auto& v : _vescpp)  // FIXME: Figure out if we keep this or not
    {
      if (v.first == can_id & 0xFF)
      {
        handleable = true;
        break;
      }
    }
    if (handleable)
      spdlog::warn(
        "[CAN::canRXcb][{}] Got Unhandled CAN frame, ID: 0x{:08X}, Data: {:pn}", _port, can_id,
        spdlog::to_hex(can_data, can_data + can_len));
  }
}

size_t CAN::addHandler(const Id id, std::function<Handler> cb)
{
  _can_handlers.emplace_back(id, cb);
  return _can_handlers.size() - 1;
}
bool CAN::removeHandler(size_t index)
{
  auto& hdlr = _can_handlers[index];
  // FIXME: Fugly, better cleanup !
  hdlr.first = 0xFFFF;
  hdlr.second = nullptr;
  return true;
}

bool CAN::write(const can_frame& fr)
{
  spdlog::trace(
    "[{}] Write CAN frame, ID: 0x{:08X}, Data: {:pn}", _port, fr.can_id,
    spdlog::to_hex(fr.data, fr.data + fr.can_dlc));
  return _can.write(fr);
}

bool CAN::write(const Id id, const uint8_t data[8], const uint8_t len)
{
  can_frame fr;
  fr.can_id = id;
  fr.can_id |= CAN_EFF_FLAG;
  fr.can_dlc = len;
  memcpy(fr.data, data, len);
  return write(fr);
}

}  // namespace vescpp::comm

namespace can
{

SocketCAN::~SocketCAN()
{
  if (this->isOpen())
  {
    this->close();
  }
}

static void* _thread_task(void* arg);

bool SocketCAN::open(
  const std::string_view& interface, std::function<void(const can_frame& frame)> rx_cb,
  int thread_priority)
{
  _sock_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (_sock_fd == -1)
  {
    spdlog::error(
      "[{}] Unable to create a CAN socket: {} (errno {})", interface, strerror(errno), errno);
    return false;
  }
  char name[16] = {};
  strncpy(name, interface.data(), interface.size());
  strncpy(_intf_req.ifr_name, name, IFNAMSIZ);
  if (ioctl(_sock_fd, SIOCGIFINDEX, &_intf_req) == -1)
  {
    spdlog::error(
      "[{}] Unable to select CAN interface: {} (errno {})", name, strerror(errno), errno);
    close();
    return false;
  }
  // Bind the socket to the network interface
  _can_addr.can_family = AF_CAN;
  _can_addr.can_ifindex = _intf_req.ifr_ifindex;
  int rc = bind(_sock_fd, reinterpret_cast<struct sockaddr*>(&_can_addr), sizeof(_can_addr));
  if (rc == -1)
  {
    spdlog::error(
      "[{}] Failed to bind socket to network interface: {} (errno {})", name, strerror(errno),
      errno);
    close();
    return false;
  }

  if (!rx_cb) return true;
  _rx_cb = std::move(rx_cb);

  // Start a separate, event-driven thread for frame reception
  _rx_run = false;
  rc = pthread_create(&_rx_thread_id, nullptr, &_thread_task, this);
  if (rc != 0)
  {
    spdlog::error("[{}] Unable to start RX thread: {} (errno {})", name, strerror(rc), rc);
    return false;
  }
  spdlog::trace("[{}] Successfully started receiver thread with ID {}", name, _rx_thread_id);
  sched_param sched{.sched_priority = thread_priority};
  pthread_setschedparam(_rx_thread_id, SCHED_FIFO, &sched);
  return true;
}

void SocketCAN::close()
{
  _rx_run = true;
  while (_rx_running)
  {
    std::this_thread::yield();
  }

  if (!isOpen())
  {
    return;
  }
  ::close(_sock_fd);
  _sock_fd = -1;
}

bool SocketCAN::isOpen() const { return (_sock_fd != -1); }

bool SocketCAN::write(const can_frame& frame) const
{
  if (!isOpen())
  {
    spdlog::error("[{}] Unable to write: Socket is not open", _intf_req.ifr_name);
    return false;
  }

  // Number of `write` retries in case of error
  constexpr int max_retries = 3;
  for (int attempt = 0; attempt < max_retries; ++attempt)
  {
    if (::write(_sock_fd, &frame, sizeof(can_frame)) != -1)
    {
      return true;
    }

    spdlog::error(
      "[{}] Unable to write CAN frame: {} (errno {}), retrying in 200us.", _intf_req.ifr_name, strerror(errno), errno);
    std::this_thread::sleep_for(200us);
  }
  spdlog::error("[{}] Max number of retries reached for `write` method.", _intf_req.ifr_name);
  return false;
}

static void* _thread_task(void* arg)
{
  //spdlog::debug("Starting thread_task");
  auto* sock = static_cast<SocketCAN*>(arg);
  sock->_rx_running = true;
  while (!sock->_rx_run)
  {
    struct timeval timeout{};
    timeout.tv_sec = 1.;

    fd_set descriptors;
    FD_ZERO(&descriptors);
    FD_SET(sock->_sock_fd, &descriptors);
    if (!select(sock->_sock_fd + 1, &descriptors, nullptr, nullptr, &timeout))
    {
      continue;
    }
    can_frame rx_frame{};
    if (read(sock->_sock_fd, &rx_frame, CAN_MTU) < 0)
    {
      continue;
    }
    sock->_rx_cb(rx_frame);
  }
  sock->_rx_running = false;
  pthread_exit(0);
  return nullptr;
}

}  // namespace can
