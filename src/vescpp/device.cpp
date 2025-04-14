
#include "vescpp/vescpp.hpp"

using namespace std::chrono_literals;

namespace vescpp
{

VESCDevice::VESCDevice(VESC::BoardId this_id, Comm* comm)
: VESCpp(this_id, comm)
{
  _fw.fw_version_major = 6;
  _fw.fw_version_minor = 2;
  _fw.hw_name = "VESCpp";
  _fw.uuid[ 0] = 0xBA; _fw.uuid[ 1] = 0xBA;
  _fw.uuid[ 2] = 0xD0; _fw.uuid[ 3] = 0x0D;
  _fw.uuid[ 4] = 0xBA; _fw.uuid[ 5] = 0xBA;
  _fw.uuid[ 6] = 0xF0; _fw.uuid[ 7] = 0x0D;
  _fw.uuid[ 8] = 0xBA; _fw.uuid[ 9] = 0xBA;
  _fw.uuid[10] = 0x00; _fw.uuid[11] = id;
  _fw.pairing_done = false;
  _fw.fw_test_version_number = 0x0;
  _fw.hw_type_vesc = ::VESC::HW_TYPE_CUSTOM_MODULE;
  _fw.custom_config = 0x00;

  if(auto* _can = dynamic_cast<comm::CAN*>(comm))
  {
    using namespace std::placeholders;

    _can->_can_handlers.emplace_back((comm::CAN::Id)(::VESC::CAN_PACKET_PING<<8)|id,
      [this](comm::CAN* can, const comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len)
      {
        spdlog::trace("[{}] Got CAN_PACKET_PING                : {:pn}", id, spdlog::to_hex(data, data+len));
        const auto& src_id = len < 1 ? 0x00 : data[0];

        uint8_t tx_data[2] = { id, _fw.hw_type_vesc};
        can->write((::VESC::CAN_PACKET_PONG<<8)|src_id, tx_data, 2);
      });
  }

  pktAddHandler(::VESC::COMM_FW_VERSION,[this](Comm* comm, const VESC::BoardId src_id, std::shared_ptr<VESC::Packet>& pkt)
  {
    if(pkt->isRequest || pkt->isAck)
    {
      spdlog::debug("[VESCDevice][{}] Handling FwVersion Request from {}", id, src_id);
      return this->send(comm, src_id, _fw, 0x01);
    }
    spdlog::debug("[VESCDevice][{}] Not Handling FwVersion from {}, when it's not a request", id, src_id);
    return false;
  });
}

}