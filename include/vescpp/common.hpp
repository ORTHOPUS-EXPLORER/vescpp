#pragma once
#include <chrono>

#include "spdlog/spdlog.h"
#include "spdlog/cfg/env.h"
#include "spdlog/fmt/bin_to_hex.h"

#include "vescpp/vesc/datatypes.h"


namespace vescpp
{
  namespace VESC
  {
    using BoardId = uint8_t;
    using HwTypeId = ::VESC::HW_TYPE;
    using PktId = ::VESC::COMM_PACKET_ID;
    constexpr PktId InvalidPktId = (::VESC::COMM_PACKET_ID)0xFFF;
  }

  using DataBuffer = std::vector<uint8_t>;
  using Time = std::chrono::system_clock;
}

#include "vescpp/vesc/utils.hpp"