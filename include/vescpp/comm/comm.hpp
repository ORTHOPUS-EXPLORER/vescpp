#pragma once

#include "vescpp/packet/packet.hpp"

namespace vescpp
{
  class Comm
  {
    public:
      virtual bool send(const VESC::Packet& pkt) = 0;
      
  };
}