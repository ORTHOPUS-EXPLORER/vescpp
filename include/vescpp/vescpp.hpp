#pragma once
#include <map>

#include "vescpp/common.hpp"
#include "vescpp/comm/comm.hpp"

#include "vescpp/packet/vesc_packets.hpp"

namespace vescpp
{

  class VESCDevice
  {
  public:
    VESCDevice(const VESC::BoardId id)
    : _id(id)
    {
    }

    bool decodePacket(const DataBuffer& buff, size_t start, size_t len)
    {
      const auto pkt_id = buff[start];
      switch(pkt_id)
      {
        case ::VESC::COMM_FW_VERSION:
          //spdlog::debug("[VESC][{}] Decoding FwVersion", id);
          return _fw.decode(buff, start, len);
        default:
          spdlog::debug("Unhandled Packet {}",pkt_id);
      }
      return false;
    }

    const VESC::BoardId& id=_id;
    const VESC::packets::FwVersion& fw=_fw; 
  private:
    VESC::BoardId _id;
    VESC::packets::FwVersion _fw;
  };

  class VESCDrive
  : public VESCDevice
  {
    public:
      VESCDrive(const VESC::BoardId id)
        : VESCDevice(id)
      {
        //spdlog::debug("[{}] New VESCDrive !", id);
      }

      bool decodePacket(const DataBuffer& buff, size_t start, size_t len)
      {
        return VESCDevice::decodePacket(buff, start, len);
      }
  };

  class VESCCustomHw
  : public VESCDevice
  {
    public:
      VESCCustomHw(const VESC::BoardId id)
        : VESCDevice(id)
      {
        //spdlog::debug("[{}] New VESCCustomHw !", id);
      }

      bool decodePacket(const DataBuffer& buff, size_t start, size_t len)
      {
        return VESCDevice::decodePacket(buff, start, len);
      }
  };
  class VESCpp
  {
  public:
    VESCpp(VESC::BoardId this_id, Comm* comm);
    ~VESCpp() = default;

    bool send(const VESC::BoardId id, const VESC::Packet* pkt);

    bool processRawPacket(const VESC::BoardId id, const DataBuffer& buff, size_t start=0, size_t len=0)
    {
      if(auto it = _devs.find(id); it != _devs.end())
        return it->second->decodePacket(buff, start, len);
      return false;
    }

    bool add_peer(const VESC::BoardId id, const VESC::HwTypeId typ);

    std::map<VESC::BoardId, std::unique_ptr<VESCDevice>> _devs;
  private:
    const VESC::BoardId _id;
    Comm* const         _comm;
  };
}