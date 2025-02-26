#pragma once

#include "vescpp/common.hpp"
#include "vescpp/comm/comm.hpp"



namespace vescpp
{
  class VESCpp
  {
  public:
    VESCpp(VESC::BoardId this_id, Comm* comm);
    ~VESCpp() = default;

    bool send(const VESC::BoardId id, const VESC::Packet* pkt);

    bool add_peer(const VESC::BoardId id, const VESC::HwTypeId typ);

  private:
    const VESC::BoardId _id;
    const Comm* _comm;
  };
}