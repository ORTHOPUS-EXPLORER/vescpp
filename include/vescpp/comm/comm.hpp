#pragma once
#include <map>

#include "vescpp/packet/packet.hpp"

namespace vescpp
{
  class VESCpp;
  class Comm
  {
    public:
      virtual bool send(const VESC::BoardId id, VESC::Packet& pkt) = 0;
      

      bool processPacket(const VESC::BoardId id, const DataBuffer& buff, size_t start=0, size_t len=0)
      {
        return false;
      }

      std::map<VESC::BoardId, VESCpp*> _vescpp;
  };
}