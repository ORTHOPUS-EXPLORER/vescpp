#include "vescpp/vescpp.hpp"
#include "vescpp/packet/vesc_packets.hpp"

namespace vescpp
{

VESCpp::VESCpp(VESC::BoardId this_id, Comm* comm)
: _id(this_id)
, _comm(comm)
{
  _comm->_vescpp[this_id] = this;
  //spdlog::debug("[{0}/0x{0:2X}] New VESCpp instance", _id);
}

bool send(const VESC::BoardId id, const VESC::Packet* pkt)
{
  return true;
}

bool VESCpp::add_peer(VESC::BoardId id, VESC::HwTypeId typ)
{
  //spdlog::debug("[{0}/0x{0:02X}] Add VESC Peer {1}: {2}", _id, id, ::VESC::HW_TYPE_s(typ));

  switch(typ)
  {
    case ::VESC::HW_TYPE_VESC:
      _devs[id] = std::make_unique<VESCDrive>(id);
      break;
    case ::VESC::HW_TYPE_CUSTOM_MODULE:
      _devs[id] = std::make_unique<VESCCustomHw>(id);
      break;
    default:
      spdlog::error("[{0}/0x{0:02X}] Unsupported Peer type {1}/{2} for ID {3}", _id, typ, ::VESC::HW_TYPE_s(typ), id);  
      return false;
  }
  VESC::packets::FwVersion pkt(true);
  _comm->send(id, pkt);

  return true; 
} 

}