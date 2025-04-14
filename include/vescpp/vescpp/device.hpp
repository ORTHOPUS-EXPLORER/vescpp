#pragma once

#include "vescpp/vescpp/base.hpp"

namespace vescpp
{

class VESCDevice
  : public VESCpp
{
public:
  VESCDevice(VESC::BoardId this_id, Comm* comm);
protected:
  VESC::packets::FwVersion _fw;
};

}
