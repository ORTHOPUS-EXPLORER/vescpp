#include "vescpp/comm/can.hpp"

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "spdlog/fmt/bin_to_hex.h"

namespace vescpp::comm 
{

CAN::CAN(const std::string_view& can_port, VESC::BoardId this_id)
: _port(can_port)
, _id(this_id)
{
  spdlog::info("[{0}][{1}/0x{1:02X}] Opening CAN Port...", _port, _id);
  //if(!_can_rx.open(_port, std::bind(&CAN::canRXcb, this, std::placeholders::_1))) 
  //{
  //  spdlog::error("[{0}][{1}/0x{1:02X}] Could not open CAN Port...", _port, _id);
  //}
  //if(!_can.open(_port, nullptr)) 
  if(!_can.open(_port, std::bind(&CAN::canRXcb, this, std::placeholders::_1))) 
  {
    spdlog::error("[{0}][{1}/0x{1:02X}] Could not open CAN Port...", _port, _id);
  }
  while(!_can._rx_running)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  spdlog::info("[{0}][{1}/0x{1:02X}] CAN Port is open !", _port, _id);

  _can_handlers.emplace_back((CAN::Id)((::VESC::CAN_PACKET_PING<<8)| _id), std::bind(&CAN::pingCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  //_can_handlers.emplace_back((CAN::Id)((::VESC::CAN_PACKET_PONG<<8)| _id), std::bind(&CAN::pongCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}


void CAN::pongCB(const Id id, const uint8_t data[8], const uint8_t len)
{
  spdlog::debug("[{0}][{1}/0x{1:02X}] Got CAN PONG: {2:pn}", _port, _id, spdlog::to_hex(data, data+len));
}

void CAN::pingCB(const Id id, const uint8_t data[8], const uint8_t len)
{
  spdlog::debug("[{0}][{1}/0x{1:02X}] Got CAN PING: {2:pn}", _port, _id, spdlog::to_hex(data, data+len));
}

void CAN::canRXcb(const can_frame& frame)
{
  auto can_id = frame.can_id&CAN_ERR_MASK;
  auto& can_data = frame.data;
  auto can_len = frame.can_dlc;
  bool handled=false;
  for(const auto& [hdlr_id,hdlr_cb]: _can_handlers)
  {
    //spdlog::debug("  CAN ID vs CAN Handler ID: {} vs {}", can_id, hdlr_id);
    if(can_id != hdlr_id || !hdlr_cb)
      continue;
    hdlr_cb(can_id, can_data, can_len);
    handled = true;
  }
  if(!handled)
  spdlog::debug("[{0}][{1}/0x{1:02X}] Got Unhandled CAN frame, ID: [{2}/0x{2:02X}], Data: {3:pn}", _port, _id, can_id, spdlog::to_hex(can_data, can_data+can_len));
  
}

bool CAN::send(const Packet& pkt)
{
  return false;
}

bool CAN::write(const can_frame& fr)
{
  //spdlog::debug("[{0}][{1}/0x{1:02X}] Write CAN ID: {2}/0x{2:02X}", _port, _id, fr.can_id);
  return _can.write(fr);
}

bool CAN::write(const Id id, const uint8_t data[8], const uint8_t len)
{
  can_frame fr;
  fr.can_id = id;
  if(fr.can_id > 0x7FF)
    fr.can_id |= CAN_EFF_FLAG;
  fr.can_dlc = len;
  memcpy(fr.data, data, len);
  return write(fr);
}

std::vector<std::pair<VESC::BoardId, VESC::HwTypeId>> CAN::scan(std::chrono::milliseconds timeout_ms)
{
  spdlog::debug("[{0}][{1}/0x{1:02X}] Scan bus for VESC boards", _port, _id);
  std::vector<std::pair<VESC::BoardId, VESC::HwTypeId>> out;
  auto& hdlr = _can_handlers.emplace_back((CAN::Id)((::VESC::CAN_PACKET_PONG<<8)| _id), 
    [this, &out](const Id id, const uint8_t data[8], const uint8_t len)
    {
      //spdlog::debug("[{0}][{1}/0x{1:02X}] Got CAN PONG: {2:pn}", _port, _id, spdlog::to_hex(data, data+len));
      out.emplace_back((VESC::BoardId)(data[0]), (VESC::HwTypeId)(data[1]));
    });
  for(uint8_t board_id=0;board_id<0xFF;board_id++)
  {
    uint8_t data = this->_id&0xFF;
    //spdlog::debug("[{0}][{1}/0x{1:02X}] Send PING to {2}", _port, _id, board_id);
    if(!write((::VESC::CAN_PACKET_PING<<8)| board_id, &data, 1))
    {
      spdlog::error("{0}][{1}/0x{1:02X}] Error when sending PING to {2}: {3}", _port, _id, board_id);
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  std::this_thread::sleep_for(timeout_ms);
  for (auto it=_can_handlers.begin(); it != _can_handlers.end();++it)
  {
    if(hdlr.first == it->first && &hdlr.second == &it->second)
    {
      //spdlog::debug("Remove handler");
      _can_handlers.erase(it);
      --it;
    }
  }
  //spdlog::debug("[{0}][{1}/0x{1:02X}] Done scanning bus for VESC boards", _port, _id);
  return out;
}
/*
def ping(self, board_id, pong_timeout=None):
#logging.debug("[CommCAN::ping]["+str(self)+"] Ping board 0x{:02X}".format(board_id))
self.write_raw(((VESCCAN.CAN_PACKET_PING<<8) | board_id),[self.can_id&0xFF])
if pong_timeout:
    t_s = time.time()
    while True:
      msg = self.read_raw(timeout=pong_timeout/10)
      if msg and  msg.arbitration_id == ((VESCCAN.CAN_PACKET_PONG<<8) | self.can_id&0xFF) and msg.data[0] == board_id:
        #logging.debug("[CommCAN::ping]["+str(self)+"] Pong from board 0x{:02X}".format(board_id))
        return True
      if time.time() - t_s > pong_timeout:
        #logging.error("[CommCAN::ping] Timeout waiting for PONG from 0x{:02X}".format(board_id))
        return False
*/
}


namespace can
{

SocketCAN::~SocketCAN()
{
  if (this->isOpen())
    this->close();
}

static void* _thread_task(void* arg);

bool SocketCAN::open(const std::string_view& interface
                     , std::function<void(const can_frame& frame)> rx_cb
                     , int thread_priority
                    )
{
  _sock_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (_sock_fd == -1)
  {
    spdlog::error("Error: Unable to create a CAN socket");
    return false;
  }
  char name[16] = {};  // avoid stringop-truncation
  strncpy(name, interface.data(), interface.size());
  strncpy(_intf_req.ifr_name, name, IFNAMSIZ);
  if (ioctl(_sock_fd, SIOCGIFINDEX, &_intf_req) == -1) // Get the index of the network interface
  {
    spdlog::error("Unable to select CAN interface {}: I/O control error", name);
    // Invalidate unusable socket
    close();
    return false;
  }
  // Bind the socket to the network interface
  _can_addr.can_family = AF_CAN;
  _can_addr.can_ifindex = _intf_req.ifr_ifindex;
  int rc = bind(_sock_fd, reinterpret_cast<struct sockaddr*>(&_can_addr), sizeof(_can_addr));
  if (rc == -1)
  {
    spdlog::error("[{}] Failed to bind socket to network interface", name);
    close();
    return false;
  }

  if(!rx_cb)
    return true;
  _rx_cb = std::move(rx_cb);

  // Start a separate, event-driven thread for frame reception
  _rx_run = false;
  rc = pthread_create(&_rx_thread_id, nullptr, &_thread_task, this);
  if (rc != 0)
  {
    spdlog::error("[{}] Unable to start RX thread", name);
    return false;
  }
  //spdlog::info("Successfully started receiver thread with ID {}", _rx_thread_id);
  sched_param sched{ .sched_priority = thread_priority };
  pthread_setschedparam(_rx_thread_id, SCHED_FIFO, &sched);
  return true;
}

void SocketCAN::close()
{
  _rx_run = true;
  while (_rx_running)
    std::this_thread::yield();

  if (!isOpen())
    return;
  ::close(_sock_fd);
  _sock_fd = -1;
}

bool SocketCAN::isOpen() const
{
  return (_sock_fd != -1);
}

bool SocketCAN::write(const can_frame& frame) const
{
  if (!isOpen())
  {
    spdlog::error("[{}] Unable to write: Socket is not open", _intf_req.ifr_name);
    return false;
  }
  if (::write(_sock_fd, &frame, sizeof(can_frame)) == -1)
  {
    spdlog::error("[{}] Unable to write. TX buffer may be full.", _intf_req.ifr_name);
    return false;
  }
  return true;
}

static void* _thread_task(void* arg)
{
  //spdlog::debug("Starting thread_task");
  auto* sock = static_cast<SocketCAN*>(arg);
  sock->_rx_running = true;
  while (!sock->_rx_run)
  {
    struct timeval timeout {};
    timeout.tv_sec = 1.;

    fd_set descriptors;
    FD_ZERO(&descriptors);
    FD_SET(sock->_sock_fd, &descriptors);
    if (!select(sock->_sock_fd + 1, &descriptors, nullptr, nullptr, &timeout))
      continue;
    can_frame rx_frame{};
    if(size_t len = read(sock->_sock_fd, &rx_frame, CAN_MTU); len < 0)
      continue;
    sock->_rx_cb(rx_frame);
  }
  sock->_rx_running = false;
  pthread_exit(0);
  return nullptr;
}

}