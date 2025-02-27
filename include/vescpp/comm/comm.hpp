#pragma once
#include <map>

#include "vescpp/packet/packet.hpp"

namespace vescpp
{
  class VESCpp;
  class Comm
  {
    public:
      virtual bool send(const VESC::BoardId src_id, const VESC::BoardId tgt_id, VESC::Packet& pkt) = 0;

      std::map<VESC::BoardId, VESCpp*> _vescpp;
  };
}