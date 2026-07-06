#pragma once
#include <chrono>
#include <future>
#include <map>
#include <memory>

#include "vescpp/vescpp/base.hpp"
#include "vescpp/vescpp/target.hpp"

namespace vescpp
{

/**
 * @ Brief Host VESC Device (like, VESC-Tool). Manages Target VESC Devices
 */
class VESCHost : public VESCpp
{
public:
  VESCHost(VESC::BoardId this_id, Comm* comm);
  virtual ~VESCHost() = default;

  template <class HwType = VESCTarget>
  std::shared_ptr<HwType> add_peer(
    VESC::BoardId board_id, VESC::HwTypeId type, bool check_firmware_version,
    std::chrono::milliseconds timeout_ms = std::chrono::milliseconds(200))
  {
    switch (type)
    {
      // Note: it's MANDATORY that _devs[board_id] exists BEFORE the firmware version check 
      // (see pktProcess in libs/orthopus_vesc/lib/vescpp/src/vescpp)
      case ::VESC::HW_TYPE_VESC:
        _devs[board_id] = std::make_shared<VESCDrive>(board_id, this);
        break;
      case ::VESC::HW_TYPE_CUSTOM_MODULE:
        _devs[board_id] = std::make_shared<HwType>(board_id, this);
        break;
      default:
        spdlog::warn(
          "[{}] Unsupported Peer type {}/{} for Peer {}", id, type, ::VESC::HW_TYPE_s(type),
          board_id);
        _devs[board_id] = std::make_shared<VESCTarget>(board_id, this);
    }

    if (check_firmware_version)
    {
      // SendRequest for FW_VERSION
      if (_devs[board_id]->request<VESC::packets::FwVersion>(timeout_ms) == nullptr)
      {
        spdlog::warn(
          "[{}] VESC Peer {} did not reply to FwVersion request, target ignored", id, board_id);
          // Note: As long as it's mandatory that _devs[board_id] exists before firmware version check, need to remove if not found
          remove_peer(board_id);
        return nullptr;
      }
    }
    spdlog::debug("[{}] Add VESC Peer {}: {}", id, board_id, ::VESC::HW_TYPE_s(type));

    return std::dynamic_pointer_cast<HwType>(_devs[board_id]);
  }

  void remove_peer(VESC::BoardId board_id)
  {
    if (auto it = _devs.find(board_id); it != _devs.end()) _devs.erase(it);
  }

  [[nodiscard]] const std::map<VESC::BoardId, std::shared_ptr<VESCTarget>> peers() const
  {
    return _devs;
  }

  template <class HwType = VESCTarget>
  std::shared_ptr<HwType> get_peer(VESC::BoardId board_id)
  {
    if (auto it = _devs.find(board_id); it != _devs.end())
      return std::dynamic_pointer_cast<HwType>(it->second);
    return nullptr;
  }

  void scanCAN(
    bool add_devices, std::chrono::milliseconds scan_timeout_ms,
    std::chrono::milliseconds ping_timeout_ms = std::chrono::milliseconds::zero());
};

}  // namespace vescpp