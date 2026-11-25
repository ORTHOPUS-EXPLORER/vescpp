#pragma once
#include <map>
#include <mutex>

#include "vescpp/packet/packet.hpp"

namespace vescpp
{
  class VESCpp; 
  class Comm
  {
    public:
      virtual ~Comm() = default;
      //virtual bool send(const VESC::BoardId src_id, const VESC::BoardId tgt_id, VESC::Packet& pkt, uint8_t send_cmd=0x00) = 0;

      void registerVESCpp(VESC::BoardId board_id, VESCpp* vescpp)
      {
        std::lock_guard lock(_vescpp_mutex);
        _vescpp[board_id] = vescpp;
      }

    protected:
      // Protects _vescpp: written by the user threads, read by the CAN RX thread
      std::mutex _vescpp_mutex;
      std::map<VESC::BoardId, VESCpp*> _vescpp;  // FIXME: Figure out if we keep this or not
  };
}