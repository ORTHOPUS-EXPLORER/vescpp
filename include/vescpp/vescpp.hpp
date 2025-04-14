#pragma once
#include <map>
#include <chrono>
#include <future>

#include "vescpp/common.hpp"
#include "vescpp/comm/can.hpp"

#include "vescpp/packet/vesc_packets.hpp"

namespace vescpp
{

class VESCpp;

class VESCDevice
{
public:
  VESCDevice(const VESC::BoardId id, VESCpp* host=nullptr, bool add_handlers=true);

  using pkt_handler_cb_t = std::function<bool(Comm* comm, const VESC::BoardId src_id, std::unique_ptr<VESC::Packet>& pkt)>;

  virtual bool pktAddHandler(VESC::PktId pkt_id, pkt_handler_cb_t cb);

  virtual bool pktProcess(Comm* comm, const VESC::BoardId src_id, std::unique_ptr<VESC::Packet>& pkt);

  bool send(VESC::Packet& pkt);
  bool sendRequest(VESC::Packet& pkt);
  
  bool sendCmd(const std::string_view& cmd);

  static constexpr auto _defaultTimeout = std::chrono::milliseconds(1000);

  template<typename PktT, typename RPktT=PktT>
  std::unique_ptr<RPktT> request(const std::chrono::milliseconds& timeout=_defaultTimeout)
  {
    PktT pkt;
    pkt.isRequest = true;
    if(this->send(pkt))
    {
      return waitFor<RPktT>(timeout);
    }
    return nullptr;
  }

  template<typename PktT>
  std::unique_ptr<PktT> waitFor(const std::chrono::milliseconds& timeout=_defaultTimeout)
  {
    _promise = {};
    _wait_for_pkt_id = PktT::id();
    auto f = _promise.get_future();
    if(f.wait_for(timeout) == std::future_status::ready)
    {
      auto r = f.get();
      if (auto pkt = dynamic_cast<PktT*>(r.get()))
      {
        r.release();
        return std::unique_ptr<PktT>(pkt);
      }
    }
    return nullptr;
  }

  const VESC::BoardId& id = _id;
  VESC::packets::FwVersion* fw() const { return _fw; }
private:
  VESC::BoardId _id;
  VESC::PktId _wait_for_pkt_id = VESC::InvalidPktId;
  std::promise<std::unique_ptr<VESC::Packet>> _promise;
protected:
  VESC::packets::FwVersion* _fw;
  VESCpp* _host;
  std::map<VESC::PktId, pkt_handler_cb_t> _pkt_handlers;
};

class VESCDrive
: public VESCDevice
{
  public:
    VESCDrive(const VESC::BoardId id, VESCpp* host=nullptr)
      : VESCDevice(id, host)
    {
      //spdlog::debug("[{}] New VESCDrive !", id);
    }
};

class VESCCustomHw
: public VESCDevice
{
  public:
    VESCCustomHw(const VESC::BoardId id, VESCpp* host=nullptr)
      : VESCDevice(id, host)
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
  bool pktProcess(Comm* comm, const VESC::BoardId src_id, std::unique_ptr<VESC::Packet>& pkt) override;

  template<class CustomHW=VESCCustomHw>
  bool add_peer(VESC::BoardId board_id, VESC::HwTypeId typ)
  {
    spdlog::debug("[{}] Add VESC Peer {}: {}", id, board_id, ::VESC::HW_TYPE_s(typ));

    VESCDevice* dev = nullptr;
    switch(typ)
    {
      case ::VESC::HW_TYPE_VESC:
        dev = new VESCDrive(board_id, this);
        break;
      case ::VESC::HW_TYPE_CUSTOM_MODULE:
        dev = new CustomHW(board_id, this);
        break;
      default:
        spdlog::warn("[{}] Unsupported Peer type {}/{} for Peer {}", id, typ, ::VESC::HW_TYPE_s(typ), board_id);  
        dev = new VESCDevice(board_id, this);
    }
    if(!dev)
      return false;
    _devs[board_id] = dev;
    
    // SendRequest for FW_VERSION
    //return dev->request<VESC::packets::FwVersion>(200ms).get() != nullptr;
    using namespace std::chrono_literals;
    VESC::packets::FwVersion pkt;
    return dev->sendRequest(pkt);
    //return dev->waitFor<VESC::packets::FwVersion>(200ms).get() != nullptr;
  } 

  template<class HwType=VESCDevice>
  HwType* get_peer(VESC::BoardId board_id)
  {
    if(auto it = _devs.find(board_id); it != _devs.end())
      return dynamic_cast<HwType*>(it->second);
    return nullptr;
  }

  // CAN 
  bool sendCAN(comm::CAN* can, const VESC::BoardId tgt_id, VESC::Packet& pkt, uint8_t send_cmd);
  std::vector<std::pair<VESC::BoardId, VESC::HwTypeId>> scanCAN(std::chrono::milliseconds timeout_ms);

  std::chrono::milliseconds msSinceLastCANPkt() const;
  std::chrono::milliseconds msSinceLastVESCkt() const;

  
  std::map<VESC::BoardId, VESCDevice*> _devs;
  protected:
  Comm* const _comm;
  const bool  _device_mode;
  
private:
  VESC::packets::FwVersion fw; 
  Time::time_point _last_can_t, _last_pkt_t;
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