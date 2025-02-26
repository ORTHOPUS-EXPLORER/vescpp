#pragma once

#include "vescpp/packet/packet.hpp"

namespace vescpp
{
  class Comm
  {
    public:
      virtual bool send(const Packet& pkt) = 0;
      
  };
}