#pragma once
#include <map>

#include "vescpp/common.hpp"
#include "vescpp/comm/can.hpp"

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

  virtual bool decodePacket(Comm* comm, const VESC::BoardId src_id, const DataBuffer& buff, size_t start, size_t len)
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
protected:
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
};
class VESCpp
: public VESCDevice
{
public:
  VESCpp(VESC::BoardId this_id, Comm* comm);
  ~VESCpp() = default;

  bool send(const VESC::BoardId id, const VESC::Packet* pkt);

  bool processRawPacket(Comm* comm, const VESC::BoardId src_id, const DataBuffer& buff, size_t start=0, size_t len=0)
  {
    //spdlog::debug("Process raw packet from {}, len {}", src_id, len);
    if(len == 2)
    {
      auto pkt_id = buff[start];
      auto payload = buff[start+1];

      DataBuffer outb;
      //spdlog::debug("Decoding request for Pkt Id: {}", pkt_id);
      switch(pkt_id)
      {
        case ::VESC::COMM_FW_VERSION:
          return comm->send(id, src_id, _fw);
        default:
          spdlog::debug("Unhandled Request for Packet {}",pkt_id);
      }
      return false;
    }
    if(auto it = _devs.find(src_id); it != _devs.end())
      return it->second->decodePacket(comm, src_id, buff, start, len);
    return false;
  }

  bool add_peer(const VESC::BoardId id, const VESC::HwTypeId typ);

  std::map<VESC::BoardId, std::unique_ptr<VESCDevice>> _local_devs;
  std::map<VESC::BoardId, std::unique_ptr<VESCDevice>> _devs;
private:
  Comm* const         _comm;

  void pingCB(comm::CAN* can, const comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len);
  
  // CAN Packets handling
  enum class PktState
  {
    Idle,
    Streaming,
    Ready
  };
  typedef struct 
  {
    Time::time_point last_t;
    uint16_t         index;
    PktState         state;
    DataBuffer       buffer;
  } pkt_rx_t;
  
  std::vector<pkt_rx_t> _rx_pkts;

    // Short packets (<6 bytes), immediate processing
  void processShortBufferCB(comm::CAN* can, const comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len);
    // Medium packets (<256 bytes), requires buffering
  void fillRXBufferCB(comm::CAN* can, const comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len);
    // Long packets    (<65536 bytes), requires buffering
  void fillRXBufferLongCB(comm::CAN* can, const comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len);
    // Medium & Long packets, process pending buffer
  void processRXBufferCB(comm::CAN* can, const comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len);
};

}