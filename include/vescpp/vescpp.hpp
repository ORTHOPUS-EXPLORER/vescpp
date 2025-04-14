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

  virtual bool decodePacket(Comm* comm, const VESC::BoardId src_id, const DataBuffer& buff, size_t start=0, size_t len=0);

  const VESC::BoardId& id = _id;
  const VESC::packets::FwVersion& fw = _fw; 
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
  VESCpp(VESC::BoardId this_id, Comm* comm, bool device_mode=true);
  ~VESCpp() = default;

  bool send(Comm* comm, const VESC::BoardId tgt_id, VESC::Packet& pkt, uint8_t send_cmd=0x00);
  bool processRawPacket(Comm* comm, const VESC::BoardId src_id, const DataBuffer& buff, size_t start=0, size_t len=0);

  template<class CustomHW=VESCCustomHw>
  bool add_peer(VESC::BoardId board_id, VESC::HwTypeId typ)
  {
    spdlog::debug("[{}] Add VESC Peer {}: {}", id, board_id, ::VESC::HW_TYPE_s(typ));

    switch(typ)
    {
      case ::VESC::HW_TYPE_VESC:
        _devs[board_id] = new VESCDrive(board_id);//std::make_unique<VESCDrive>(board_id);
        break;
      case ::VESC::HW_TYPE_CUSTOM_MODULE:
        _devs[board_id] = new CustomHW(board_id);//std::make_unique<CustomHW>(board_id);
        break;
      default:
        spdlog::warn("[{}] Unsupported Peer type {}/{} for Peer {}", id, typ, ::VESC::HW_TYPE_s(typ), board_id);  
        _devs[board_id] = new VESCDevice(board_id);//std::make_unique<VESCDevice>(board_id);
    }
    // SendRequest for FW_VERSION
    VESC::packets::FwVersion pkt(true);
    send(_comm, board_id, pkt);
    return true;
  } 

  // CAN 
  bool sendCAN(comm::CAN* can, const VESC::BoardId tgt_id, VESC::Packet& pkt, uint8_t send_cmd);
  std::vector<std::pair<VESC::BoardId, VESC::HwTypeId>> scanCAN(std::chrono::milliseconds timeout_ms);

  std::map<VESC::BoardId, VESCDevice*> _devs;
protected:
  Comm* const _comm;
  const bool  _device_mode;
  
private:

  // CAN
  // Callbacks
  void pingCB(comm::CAN* can, const comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len);
  void pongCB(comm::CAN* can, const comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len);
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