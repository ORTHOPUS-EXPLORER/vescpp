
#include "vescpp/vescpp.hpp"

using namespace std::chrono_literals;

namespace vescpp
{

VESCHost::VESCHost(VESC::BoardId this_id, Comm* comm)
: VESCpp(this_id, comm)
{
}

void VESCHost::scanCAN(bool add_devices, std::chrono::milliseconds scan_timeout_ms, std::chrono::milliseconds ping_timeout_ms)
{
  if(ping_timeout_ms == std::chrono::milliseconds::zero())
    ping_timeout_ms = scan_timeout_ms;
  if(add_devices)
  {
    std::vector<std::pair<VESC::BoardId, VESC::HwTypeId>> out;
    spdlog::debug("[{0}/0x{0:02X}] Scan CAN and add devices", this->id);
    VESCpp::scanCAN(scan_timeout_ms, [this_id=id,&out](VESC::BoardId id, VESC::HwTypeId typ)
    {
      spdlog::debug("[{0}/0x{0:02X}] CAN_PONG from {1}/0x{1:02X}", this_id, id);
      out.emplace_back(id,typ);
    });
    for(const auto& [id, typ]: out)
    {
      if(auto v = this->add_peer(id,typ, ping_timeout_ms); v != nullptr)
      {
        const auto& fw = v->fw();
        spdlog::debug("[{0}/0x{0:02X}] FW version: {1}.{2} - HW: {3:<15s} - UUID: 0x {4:spn}", id, fw->fw_version_major, fw->fw_version_minor,  fw->hw_name.c_str(), spdlog::to_hex(fw->uuid));
      }
    }
  }
  else
    VESCpp::scanCAN(scan_timeout_ms);
}

}
