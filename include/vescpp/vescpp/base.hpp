#pragma once
#include <map>
#include <chrono>
#include <future>

#include "vescpp/common.hpp"
#include "vescpp/comm/can.hpp"

#include "vescpp/packet/vesc_packets.hpp"

namespace vescpp
{

  /**
   * @brief Base VESC Device, has an ID and handle packets and has basic operations
   */
class VESCBase
{
public:
  VESCBase(const VESC::BoardId board_id, VESCpp* host=nullptr);

  using pkt_handler_cb_t = std::function<bool(Comm* comm, const VESC::BoardId src_id, std::shared_ptr<VESC::Packet>& pkt)>;

  bool pktAddHandler(VESC::PktId pkt_id, pkt_handler_cb_t cb);

  virtual bool pktProcess(Comm* comm, const VESC::BoardId src_id, std::shared_ptr<VESC::Packet>& pkt);

  static constexpr auto _defaultTimeout = std::chrono::milliseconds(1000);

  bool send(VESC::Packet& pkt);
  bool sendRequest(VESC::Packet& pkt);

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

  std::shared_ptr<vescpp::VESC::RawPacket> waitFor(VESC::PktId pkt_id, const std::chrono::milliseconds& timeout=_defaultTimeout)
  {
    _promise = {};
    _wait_for_pkt_id = pkt_id;
    spdlog::trace("Start waiting for packet ID: {}/{}", _wait_for_pkt_id, fmt::ptr(&_wait_for_pkt_id));
    auto f = _promise.get_future();
    if(f.wait_for(timeout) == std::future_status::ready)
    {
        return std::dynamic_pointer_cast<vescpp::VESC::RawPacket>(f.get());
    }
    return nullptr;
  }

  const VESC::BoardId id;
//private:
//protected:
  std::promise<std::shared_ptr<VESC::Packet>> _promise;
  VESC::PktId _wait_for_pkt_id = VESC::InvalidPktId;
protected:
  VESCpp* _host;

  std::map<VESC::PktId, pkt_handler_cb_t> _pkt_handlers;
};

/**
 * Generic VESC Target
 */
class VESCTarget
: public VESCBase
{
public:
  VESCTarget(const VESC::BoardId id, VESCpp* host=nullptr);

  const std::shared_ptr<VESC::packets::FwVersion>& fw() { return _fw; };

  bool sendCmd(const std::string_view& cmd, std::chrono::milliseconds delay_after_send=std::chrono::milliseconds(5));

  bool flashFirmware(const std::string& fw_file, bool isBootloader, bool fwdCan, bool isLzo);

protected:
  std::shared_ptr<VESC::packets::FwVersion> _fw;
};


/**
 * @ Brief Base class for CAN
 */
class VESCpp
: public VESCBase
{
public:
  VESCpp(VESC::BoardId this_id, Comm* comm);
  ~VESCpp() = default;

  bool send(Comm* comm, const VESC::BoardId tgt_id, VESC::Packet& pkt, uint8_t send_cmd=0x00);
  bool processRawPacket(Comm* comm, const VESC::BoardId src_id, const DataBuffer& buff, size_t start=0, size_t len=0);
  bool pktProcess(Comm* comm, const VESC::BoardId src_id, std::shared_ptr<VESC::Packet>& pkt) override;

  // CAN
  bool sendCAN(comm::CAN* can, const VESC::BoardId tgt_id, VESC::Packet& pkt, uint8_t send_cmd);
  void scanCAN(std::chrono::milliseconds timeout_ms, std::function<void(VESC::BoardId, VESC::HwTypeId)> cb=nullptr);

  std::chrono::milliseconds msSinceLastCANPkt() const;
  std::chrono::milliseconds msSinceLastVESCkt() const;

  std::map<VESC::BoardId, std::shared_ptr<VESCTarget>> _devs;
  protected:
  Comm* const _comm;

  static constexpr std::chrono::microseconds _tx_delay_us{10};

private:
  Time::time_point _last_can_t, _last_pkt_t;
  // CAN
  // Callbacks

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
