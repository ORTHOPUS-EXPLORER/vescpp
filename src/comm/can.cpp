#include "vescpp/comm/can.hpp"

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "vescpp/vescpp.hpp"

namespace vescpp::comm 
{

CAN::CAN(const std::string_view& can_port)
: _port(can_port)
{
  spdlog::info("[{}] Opening CAN Port...", _port);
  //if(!_can_rx.open(_port, std::bind(&CAN::canRXcb, this, std::placeholders::_1))) 
  //{
  //  spdlog::error("[{}] Could not open CAN Port...", _port);
  //}
  //if(!_can.open(_port, nullptr)) 
  if(!_can.open(_port, std::bind(&CAN::canRXcb, this, std::placeholders::_1))) 
  {
    spdlog::error("[{}] Could not open CAN Port...", _port);
  }
  while(!_can._rx_running)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  spdlog::info("[{}] CAN Port is open !", _port);

}


void CAN::canRXcb(const can_frame& frame)
{
  auto can_id = frame.can_id&CAN_ERR_MASK;
  auto& can_data = frame.data;
  auto can_len = frame.can_dlc;
  bool handled=false;;
  for(const auto& [hdlr_id,hdlr_cb]: _can_handlers)
  {
    //spdlog::debug("  CAN ID vs CAN Handler ID: {} vs {}", can_id, hdlr_id);
    if(can_id != hdlr_id || !hdlr_cb)
      continue;

    hdlr_cb(this, can_id, can_data, can_len);
    handled = true;
  }
  if(!handled)
  {
    //spdlog::debug("[{0}] Got Unhandled CAN frame, ID: [{1}/0x{1:02X}], Data: {2:pn}", _port, can_id, spdlog::to_hex(can_data, can_data+can_len));
  }  
}

bool CAN::send(const VESC::BoardId src_id, const VESC::BoardId tgt_id, VESC::Packet& pkt)
{
  DataBuffer pktbuf, buf;
  if(!pkt.encode(pktbuf))
  {
    spdlog::debug("[{}=>{}] Could not encode Packet {} from {} to {}", src_id, tgt_id, pkt.id);
    return false;
  }
  auto sz = pktbuf.size();
  if(sz <= 6)
  {
    //spdlog::debug("[{}>{}] Send CAN_PACKET_PROCESS_SHORT_BUFFER for {}", src_id, tgt_id, pkt.id);
    buf.push_back(src_id);   
    buf.push_back(0x00); // Process packet at receiver
    for(const auto& c: pktbuf)
      buf.push_back(c);
    return write((::VESC::CAN_PACKET_PROCESS_SHORT_BUFFER<<8)|tgt_id,buf.data(),buf.size());
  }
  else
  {
    spdlog::debug("[{}=>{}] Send CAN_PACKET_FILL_RX_BUFFER for Pkt {}, len {}", src_id, tgt_id, pkt.id, sz);
    spdlog::debug("[{}=>{}] Send CAN_PACKET_FILL_RX_BUFFER_LONG for Pkt {}, len {}", src_id, tgt_id, pkt.id, sz);
    spdlog::debug("[{}=>{}] Send CAN_PACKET_PROCESS_RX_BUFFER for Pkt {}, len {}", src_id, tgt_id, pkt.id, sz);
    // FIXME: see vesc_interface.cpp:3251
    return true;
  }
  
  return false;
}

bool CAN::write(const can_frame& fr)
{
  //spdlog::debug("[{0}] Write CAN ID: {2}/0x{2:02X}", _port, fr.can_id);
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

std::vector<std::pair<VESC::BoardId, VESC::HwTypeId>> CAN::scan(const VESC::BoardId src_id, std::chrono::milliseconds timeout_ms)
{
  //spdlog::debug("[{0}] Scan bus for VESC boards", _port);
  std::vector<std::pair<VESC::BoardId, VESC::HwTypeId>> out;
  auto& hdlr = _can_handlers.emplace_back(((CAN::Id)::VESC::CAN_PACKET_PONG<<8)|src_id, 
    [this, &out](CAN* can, const Id can_id, const uint8_t data[8], const uint8_t len)
    {
      //spdlog::debug("[{0}] Got CAN PONG: {1:pn}", _port, spdlog::to_hex(data, data+len));
      out.emplace_back((VESC::BoardId)(data[0]), (VESC::HwTypeId)(data[1]));
    });
  for(uint8_t board_id=0;board_id<0xFF;board_id++)
  {
    uint8_t data = src_id&0xFF;
    //spdlog::debug("[{0}] Send PING to {2}", _port, board_id);
    if(!write((::VESC::CAN_PACKET_PING<<8)| board_id, &data, 1))
    {
      spdlog::error("{0}] Error when sending PING to {2}: {3}", _port, board_id);
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
  //spdlog::debug("[{0}] Done scanning bus for VESC boards", _port);
  return out;
}

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