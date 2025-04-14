#pragma once
#include <map>
#include <chrono>
#include <future>

#include "vescpp/common.hpp"
#include "vescpp/vescpp/base.hpp"

#include "vescpp/packet/vesc_packets.hpp"

namespace vescpp
{

// VESCTarget is defined in base.hpp

class VESCDrive
: public VESCTarget
{
  public:
    VESCDrive(const VESC::BoardId id, VESCpp* host=nullptr)
      : VESCTarget(id, host)
    {
      //spdlog::debug("[{}] New VESCDrive !", id);
    }
};

class VESCCustomHw
: public VESCTarget
{
  public:
    VESCCustomHw(const VESC::BoardId id, VESCpp* host=nullptr)
      : VESCTarget(id, host)
    {
      //spdlog::debug("[{}] New VESCCustomHw !", id);
    }
};

}
