#pragma once
#include <map>
#include <chrono>
#include <future>

#include "vescpp/vescpp/base.hpp"
#include "vescpp/vescpp/target.hpp"

namespace vescpp
{

/**
 * @ Brief Host VESC Device (like, VESC-Tool). Manages Target VESC Devices
 */
class VESCHost
  : public VESCpp
{
public:
  VESCHost(VESC::BoardId this_id, Comm* comm);

  template<class HwType=VESCTarget>
  std::shared_ptr<HwType> add_peer(VESC::BoardId board_id, VESC::HwTypeId typ, std::chrono::milliseconds timeout_ms=std::chrono::milliseconds(200))
  {
    VESCTarget* dev = nullptr;
    switch(typ)
    {
      case ::VESC::HW_TYPE_VESC:
        dev = new VESCDrive(board_id, this);
        break;
      case ::VESC::HW_TYPE_CUSTOM_MODULE:
        dev = new HwType(board_id, this);
        break;
      default:
        spdlog::warn("[{}] Unsupported Peer type {}/{} for Peer {}", id, typ, ::VESC::HW_TYPE_s(typ), board_id);
        dev = new VESCTarget(board_id, this);
    }
    _devs[board_id] = std::shared_ptr<VESCTarget>(dev);
    // SendRequest for FW_VERSION
    if(dev->request<VESC::packets::FwVersion>(timeout_ms) != nullptr)
    {
      spdlog::debug("[{}] Add VESC Peer {}: {}", id, board_id, ::VESC::HW_TYPE_s(typ));
      return std::dynamic_pointer_cast<HwType>(_devs[board_id]);
    }
    spdlog::warn("[{}] VESC Peer {} did not reply to FwVersion request, ignore it", id, board_id);
    _devs.extract(board_id);
    return nullptr;
  }

  const std::map<VESC::BoardId, std::shared_ptr<VESCTarget>> peers() const
  {
    return _devs;
  }

  template<class HwType=VESCTarget>
  std::shared_ptr<HwType> get_peer(VESC::BoardId board_id)
  {
    if(auto it = _devs.find(board_id); it != _devs.end())
      return std::dynamic_pointer_cast<HwType>(it->second);
    return nullptr;
  }

  void scanCAN(bool add_devices, std::chrono::milliseconds timeout_ms);

};

}