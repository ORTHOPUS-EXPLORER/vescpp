#pragma once
#include <map>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>

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

  bool pktAddHandler(VESC::PktId pkt_id, pkt_handler_cb_t cb, bool replace=false);

  virtual bool pktProcess(Comm* comm, const VESC::BoardId src_id, std::shared_ptr<VESC::Packet>& pkt);

  static constexpr auto _defaultTimeout = std::chrono::milliseconds(1000);

  bool send(VESC::Packet& pkt);
  bool sendRequest(VESC::Packet& pkt);

  template<typename PktT, typename RPktT=PktT>
  std::shared_ptr<RPktT> request(const std::chrono::milliseconds& timeout=_defaultTimeout)
  {
    PktT pkt;
    pkt.isRequest = true;
    // Start waiting BEFORE sending, otherwise a fast reply could be missed
    startWait(RPktT::id());
    if(!this->send(pkt))
    {
      cancelWait();
      return nullptr;
    }
    return std::dynamic_pointer_cast<RPktT>(endWait(timeout));
  }

  // Wait for a packet received from now on. Use `request` to send a packet and wait for its reply
  template<typename PktT>
  std::shared_ptr<PktT> waitFor(const std::chrono::milliseconds& timeout=_defaultTimeout)
  {
    startWait(PktT::id());
    return std::dynamic_pointer_cast<PktT>(endWait(timeout));
  }

  std::shared_ptr<vescpp::VESC::RawPacket> waitFor(VESC::PktId pkt_id, const std::chrono::milliseconds& timeout=_defaultTimeout)
  {
    startWait(pkt_id);
    return std::dynamic_pointer_cast<vescpp::VESC::RawPacket>(endWait(timeout));
  }

  // Give the packet to the thread waiting for this type of packet. Returns false if there is none
  bool notifyWait(VESC::PktId pkt_id, const std::shared_ptr<VESC::Packet>& pkt);

  const VESC::BoardId id;
protected:
  VESCpp* _host;

  std::map<VESC::PktId, pkt_handler_cb_t> _pkt_handlers;

private:
  // Wait for a packet, received by the CAN RX thread. Only one thread can wait at a time
  void startWait(VESC::PktId pkt_id);
  void cancelWait();
  // Blocks until the packet is received, returns nullptr on timeout
  std::shared_ptr<VESC::Packet> endWait(const std::chrono::milliseconds& timeout);

  // Protects the 2 members below
  std::mutex _wait_mutex;
  std::condition_variable _wait_cv;
  // InvalidPktId: nobody is waiting
  VESC::PktId _wait_for_pkt_id = VESC::InvalidPktId;
  std::shared_ptr<VESC::Packet> _wait_reply;
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

  // Text command
  bool sendCmd(const std::string_view& cmd, std::chrono::milliseconds delay_after_send=std::chrono::milliseconds(5));

  bool flashFirmware(const std::string& fw_file, bool isBootloader, bool fwdCan, bool isLzo);
  bool eraseFirmware(bool isBootloader, bool fwdCan, bool isLzo, std::chrono::milliseconds delay_after_send, size_t fwSize);
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

  // Devices are used from several threads (CAN RX, TX, user): only access them with these methods.
  // Returned devices stay valid even if they are removed meanwhile by another thread.
  [[nodiscard]] std::shared_ptr<VESCTarget> getDevice(VESC::BoardId board_id) const;
  [[nodiscard]] std::map<VESC::BoardId, std::shared_ptr<VESCTarget>> getDevices() const;
  protected:
  void setDevice(VESC::BoardId board_id, std::shared_ptr<VESCTarget> dev);
  void removeDevice(VESC::BoardId board_id);

  Comm* const _comm;

  static constexpr std::chrono::microseconds _tx_delay_us{10};

private:
  // Never hold the mutex while calling a device method or writing on CAN
  mutable std::mutex _devs_mutex;
  std::map<VESC::BoardId, std::shared_ptr<VESCTarget>> _devs;

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
