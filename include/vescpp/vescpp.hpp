#pragma once

#include "vescpp/common.hpp"
#include "vescpp/comm/comm.hpp"

namespace vescpp
{
  class VESCpp
  {
  public:
    VESCpp(VESC::BoardId this_id, Comm* comm);

    bool add_peer(VESC::BoardId id);

  private:
    const VESC::BoardId _id;
    const Comm* _comm;
  };
}