#pragma once
#include <chrono>

#include <fmt/std.h>
#include "spdlog/spdlog.h"
#include "spdlog/cfg/env.h"
#include "spdlog/fmt/bin_to_hex.h"

#include "vescpp/vesc/datatypes.h"


namespace vescpp
{
  namespace VESC
  {
    using BoardId = uint8_t;
    constexpr BoardId InvalidBoardId = 0x00;
    using HwTypeId = ::VESC::HW_TYPE;
    using PktId = ::VESC::COMM_PACKET_ID;
    constexpr PktId InvalidPktId = (::VESC::COMM_PACKET_ID)0xFFF;
  }

  using DataBuffer = std::vector<uint8_t>;
  using Time = std::chrono::system_clock;
}

template<>
struct fmt::formatter<vescpp::VESC::PktId> : fmt::formatter<std::string>
{
    auto format(vescpp::VESC::PktId v, format_context &ctx) const -> decltype(ctx.out())
    {
        return fmt::format_to(ctx.out(), "{:d}", (uint8_t)v);
    }
};

template<>
struct fmt::formatter<vescpp::VESC::HwTypeId> : fmt::formatter<std::string>
{
    auto format(vescpp::VESC::HwTypeId v, format_context &ctx) const -> decltype(ctx.out())
    {
        return fmt::format_to(ctx.out(), "{:d}", (uint8_t)v);
    }
};

#include "vescpp/vesc/utils.hpp"