#pragma once
#include <map>

#include "vescpp/packet/packet.hpp"

namespace vescpp
{
  class VESCpp; 
  class Comm
  {
    public:
      virtual ~Comm() = default;
      //virtual bool send(const VESC::BoardId src_id, const VESC::BoardId tgt_id, VESC::Packet& pkt, uint8_t send_cmd=0x00) = 0;

      std::map<VESC::BoardId, VESCpp*> _vescpp;  // FIXME: Figure out if we keep this or not
  };
}