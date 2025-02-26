#pragma once
#include <CanDriver.hpp>

#include "vescpp/common.hpp"
#include "vescpp/comm/comm.hpp"

namespace vescpp::comm
{

class CAN
  : public Comm
{
public:
  CAN(const std::string_view& can_port, VESC::BoardId this_id);
  ~CAN();
  bool send(const Packet& pkt) override;
  
  std::vector<VESC::BoardId> scan(std::chrono::milliseconds timeout_ms = std::chrono::milliseconds(1000));

private:
  const std::string   _port;
  const VESC::BoardId _id;
  sockcanpp::CanDriver _can;
  std::thread _rx_th;
  std::atomic_bool _run_rx;
};

}