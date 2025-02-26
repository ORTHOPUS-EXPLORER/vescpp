#pragma once
#include <string_view>

#include "vescpp/common.hpp"

#include "vescpp/vesc/datatypes.h"

namespace VESC
{

std::string_view HW_TYPE_s(const HW_TYPE id);
uint16_t crc16(const vescpp::DataBuffer& buf, size_t start=0, size_t len=0);

}