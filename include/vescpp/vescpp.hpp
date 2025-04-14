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
  VESCDevice(const VESC::BoardId board_id, VESCpp* host=nullptr, bool add_handlers=true);

  using pkt_handler_cb_t = std::function<bool(Comm* comm, const VESC::BoardId src_id, std::shared_ptr<VESC::Packet>& pkt)>;

  virtual bool pktAddHandler(VESC::PktId pkt_id, pkt_handler_cb_t cb);

  virtual bool pktProcess(Comm* comm, const VESC::BoardId src_id, std::shared_ptr<VESC::Packet>& pkt);

  bool send(VESC::Packet& pkt);
  bool sendRequest(VESC::Packet& pkt);
  
  bool sendCmd(const std::string_view& cmd, std::chrono::milliseconds delay_after_send=std::chrono::milliseconds(5));

  static constexpr auto _defaultTimeout = std::chrono::milliseconds(1000);

  template<typename PktT, typename RPktT=PktT>
  std::shared_ptr<RPktT> request(const std::chrono::milliseconds& timeout=_defaultTimeout)
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
  std::shared_ptr<PktT> waitFor(const std::chrono::milliseconds& timeout=_defaultTimeout)
  {
    _promise = {};
    _wait_for_pkt_id = PktT::id();
    auto f = _promise.get_future();
    if(f.wait_for(timeout) == std::future_status::ready)
    {
        return std::dynamic_pointer_cast<PktT>(f.get());
    }
    return nullptr;
  }

  const VESC::BoardId id;
  const std::shared_ptr<VESC::packets::FwVersion>& fw() { return _fw; };
private:
  VESC::PktId _wait_for_pkt_id = VESC::InvalidPktId;
  std::promise<std::shared_ptr<VESC::Packet>> _promise;
protected:
  VESCpp* _host;
  std::shared_ptr<VESC::packets::FwVersion> _fw;
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
  bool pktProcess(Comm* comm, const VESC::BoardId src_id, std::shared_ptr<VESC::Packet>& pkt) override;

  template<class HwType=VESCDevice>
  std::shared_ptr<VESCDevice> add_peer(VESC::BoardId board_id, VESC::HwTypeId typ, std::chrono::milliseconds timeout_ms=std::chrono::milliseconds(200))
  {
    

    VESCDevice* dev = nullptr;
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
        dev = new VESCDevice(board_id, this);
    }
    _devs[board_id] = std::shared_ptr<VESCDevice>(dev);
    // SendRequest for FW_VERSION
    if(dev->request<VESC::packets::FwVersion>(timeout_ms) != nullptr)
    {
      spdlog::debug("[{}] Add VESC Peer {}: {}", id, board_id, ::VESC::HW_TYPE_s(typ));
      return std::dynamic_pointer_cast<HwType>(_devs[board_id]);
    }
    spdlog::warn("[{}] VESC Peer {} did not reply to FwVersion request, ignore it", id, board_id);
    _devs[board_id] = nullptr;
    return nullptr;
  } 

  template<class HwType=VESCDevice>
  std::shared_ptr<HwType> get_peer(VESC::BoardId board_id)
  {
    if(auto it = _devs.find(board_id); it != _devs.end())
      return std::dynamic_pointer_cast<HwType>(it->second);
    return nullptr;
  }

  // CAN 
  bool sendCAN(comm::CAN* can, const VESC::BoardId tgt_id, VESC::Packet& pkt, uint8_t send_cmd);
  std::vector<std::pair<VESC::BoardId, VESC::HwTypeId>> scanCAN(std::chrono::milliseconds timeout_ms);

  std::chrono::milliseconds msSinceLastCANPkt() const;
  std::chrono::milliseconds msSinceLastVESCkt() const;
  
  std::map<VESC::BoardId, std::shared_ptr<VESCDevice>> _devs;
  protected:
  Comm* const _comm;
  const bool  _device_mode;

  static constexpr std::chrono::microseconds _tx_delay_us{10};
  
private:
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