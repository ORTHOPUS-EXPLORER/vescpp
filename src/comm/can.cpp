#include "vescpp/comm/can.hpp"

#include <exceptions/CanException.hpp>

using sockcanpp::CanDriver;
using sockcanpp::CanId;
using sockcanpp::CanMessage;
using sockcanpp::exceptions::CanException;

namespace vescpp::comm 
{

CAN::CAN(const std::string_view& can_port, VESC::BoardId this_id)
: _port(can_port)
, _id(this_id)
, _can(_port.data(), CAN_RAW, 0x00)
, _rx_th([this](void)
{
  auto _can_rx = CanDriver(_port.data(), CAN_RAW, 0x00);
  _run_rx = true;
  while(_run_rx)
  {
    try
    {
      const auto msg = _can_rx.readMessage();
      const auto can_id = (uint16_t)msg.getCanId();
      const auto msg_id = can_id >> 8;
      const VESC::BoardId board_id = can_id & 0x0FF;
      const auto& data = msg.getFrameData();
      if(board_id == _id)
        spdlog::debug("[{0}][{1}/0x{1:02X}] Got Msg {2} from {3}/0x{3:02X}: 0x{4:02X}, 0x{4:02X}", _port, _id, msg_id, (VESC::BoardId)data[0], data[1]);
    }
    catch(CanException& e)
    {
      //spdlog::error("{0}][{1}/0x{1:02X}] Exception when reading from bus: {2}", _port, _id, e.what());
    }
    //std::this_thread::sleep_for(std::chrono::microseconds(10));
  }
})
{
  spdlog::info("[{0}][{1}/0x{1:02X}] Opening CAN Port...", _port, _id);
}

CAN::~CAN()
{
  //_can.uninitialiseSocketCan();
  _run_rx = false;
  _rx_th.join();
}

bool CAN::send(const Packet& pkt)
{
  return false;
}

std::vector<VESC::BoardId> CAN::scan(std::chrono::milliseconds timeout_ms)
{
  spdlog::debug("[{0}][{1}/0x{1:02X}] Scan bus for VESC boards", _port, _id);
  for(uint8_t board_id=0;board_id<0xFF;board_id++)
  {
    CanId can_id = (::VESC::CAN_PACKET_PING<<8)| board_id;
    std::string data = "";
    data[0] = this->_id&0xFF;
    try
    {
      auto sentByteCount = _can.sendMessage({can_id, data});
      //spdlog::debug("[{0}][{1}/0x{1:02X}] Sent PING to {2}, {3} bytes", _port, _id, board_id, sentByteCount);
    }
    catch(CanException& e)
    {
      spdlog::error("{0}][{1}/0x{1:02X}] Exception when sending PING to {2}: {3}", _port, _id, board_id, e.what());
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  std::this_thread::sleep_for(timeout_ms);
  spdlog::debug("[{0}][{1}/0x{1:02X}] Done scanning bus for VESC boards", _port, _id);
  return {};
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
