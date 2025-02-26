#pragma once
//#include <CanDriver.hpp>

#include <linux/can.h>

#include <net/if.h>
#include <thread>
#include <functional>

#include "vescpp/common.hpp"
#include "vescpp/comm/comm.hpp"

namespace can
{

class SocketCAN
{
private:
  ifreq _intf_req{};
  sockaddr_can _can_addr{};
  pthread_t _rx_thread_id{};

public:
  using rx_cb_t = void(const can_frame& frame);
  // These need to be public for _thread_task in .cpp
  int _sock_fd                        = -1;
  std::function<rx_cb_t>  _rx_cb      = nullptr;
  std::atomic_bool        _rx_run     = false;
  std::atomic_bool        _rx_running = false;

  SocketCAN() = default;
  ~SocketCAN();

  bool open(const std::string_view& interface
           , std::function<rx_cb_t> rx_cb
           , int thread_priority=50);
  void close();
  bool isOpen() const;
  bool write(const can_frame& frame) const;
  bool read(can_frame& frame/*, std::chrono::milliseconds timeout_ms*/);
};

}

namespace vescpp::comm
{

class CAN
  : public Comm
{
public:
  using Id = uint16_t;
  using Handler = void(const Id can_id, const uint8_t data[8], const uint8_t len);
  CAN(const std::string_view& can_port, VESC::BoardId this_id);
  ~CAN() = default;
  bool send(const Packet& pkt) override;
  
  std::vector<std::pair<VESC::BoardId, VESC::HwTypeId>> scan(std::chrono::milliseconds timeout_ms = std::chrono::milliseconds(1000));

  bool write(const can_frame& frame);
  bool write(const Id id, const uint8_t data[8], const uint8_t len);

private:
  const std::string   _port;
  const VESC::BoardId _id;
  can::SocketCAN _can;//, _can_rx;
  void canRXcb(const can_frame& frame);
  std::vector<std::pair<Id, std::function<Handler>>> _can_handlers;

  void pingCB(const Id can_id, const uint8_t data[8], const uint8_t len);
  void pongCB(const Id can_id, const uint8_t data[8], const uint8_t len);
};

}