#include "vescpp/vesc/utils.hpp"

namespace VESC
{

std::string_view HW_TYPE_s(const HW_TYPE id)
{
    switch(id)
    {
        case HW_TYPE_VESC:
            return "VESC";
        case HW_TYPE_VESC_BMS:
            return "BMS";
        case HW_TYPE_CUSTOM_MODULE:
            return "Custom";
        default:
            return "Unkown";
    }
}

}