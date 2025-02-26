#include "vescpp/vescpp.hpp"

namespace vescpp
{

VESCpp::VESCpp(VESC::BoardId this_id, Comm* comm)
: _id(this_id)
, _comm(comm)
{
  //spdlog::debug("[{0}/0x{0:2X}] New VESCpp instance", _id);
}

bool VESCpp::add_peer(VESC::BoardId id, VESC::HwTypeId typ)
{
  spdlog::debug("[{0}/0x{0:2X}] Add VESC Peer {1}: {2}", _id, id, ::VESC::HW_TYPE_s(typ));
  return true; 
} 

}