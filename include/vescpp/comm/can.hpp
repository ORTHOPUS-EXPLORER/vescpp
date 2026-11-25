#pragma once
//#include <CanDriver.hpp>

#include <linux/can.h>

#include <net/if.h>
#include <atomic>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>

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
  using Handler = void(CAN* can, const Id can_id, const uint8_t data[8], const uint8_t len);
  CAN(const std::string_view& can_port);
  ~CAN() = default;

  bool write(const can_frame& frame);
  bool write(const Id id, const uint8_t data[8], const uint8_t len);

  // Handlers are called from the CAN RX thread.
  // Warning: do not call addHandler/removeHandler from a handler (deadlock)
  size_t addHandler(const Id id, std::function<Handler> cb);
  // Once it returns, the handler is not running and will never be called again.
  // Returns false if there is no such handler
  bool removeHandler(size_t handler_id);

private:
  // Held by the RX thread while it calls the handlers
  std::mutex _can_handlers_mutex;
  size_t _next_handler_id = 0;
  // Sorted by handler id, so called in the order they were added
  std::map<size_t, std::pair<Id, std::function<Handler>>> _can_handlers;

  const std::string   _port;
  can::SocketCAN _can;//, _can_rx;
  void canRXcb(const can_frame& frame);
};

}