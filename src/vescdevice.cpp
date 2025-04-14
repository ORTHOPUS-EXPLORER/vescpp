#include "vescpp/vescpp.hpp"

namespace vescpp
{

VESCDevice::VESCDevice(const VESC::BoardId id, VESCpp* host, bool add_handlers)
  : _id(id)
  , _fw(nullptr)
  , _host(host)
{
  if(!add_handlers)
    return;

  pktAddHandler(::VESC::COMM_FW_VERSION,[this](Comm* comm, const VESC::BoardId src_id, std::unique_ptr<VESC::Packet>& pkt)
  {
    //spdlog::debug("[VESC][{}/{}] Handling FwVersion", this->id, src_id);
    _fw = dynamic_cast<VESC::packets::FwVersion*>(pkt.get());
    pkt.release();
    return true;
  });
  pktAddHandler(::VESC::COMM_PRINT,[this](Comm* comm, const VESC::BoardId src_id, std::unique_ptr<VESC::Packet>& pkt)
  {
    auto* ppkt = dynamic_cast<VESC::packets::Print*>(pkt.get());
    //spdlog::debug("[VESC][{}/{}] Handling Print", this->id, src_id);
    spdlog::info("[VESC][{}]<= {}", this->id, ppkt->str);
    return true;
  });

  pktAddHandler(::VESC::COMM_SET_MCCONF,[this](Comm* comm, const VESC::BoardId src_id, std::unique_ptr<VESC::Packet>& pkt)
  {
    //auto* ppkt = dynamic_cast<VESC::packets::SetMCConf*>(pkt.get());
    if(pkt->isAck)
    {
      spdlog::info("[VESC][{}<={}] COMM_SET_MCCONF OK", this->id, src_id);
      return true;
    }
    return false;
  });
  pktAddHandler(::VESC::COMM_SET_APPCONF,[this](Comm* comm, const VESC::BoardId src_id, std::unique_ptr<VESC::Packet>& pkt)
  {
    //auto* ppkt = dynamic_cast<VESC::packets::SetAppConf*>(pkt.get());
    if(pkt->isAck)
    {
      spdlog::info("[VESC][{}<={}] COMM_SET_APPCONF OK", this->id, src_id);
      return true;
    }
    return false;
  });
  pktAddHandler(::VESC::COMM_SET_CUSTOM_CONFIG,[this](Comm* comm, const VESC::BoardId src_id, std::unique_ptr<VESC::Packet>& pkt)
  {
    //auto* ppkt = dynamic_cast<VESC::packets::SetAppConf*>(pkt.get());
    if(pkt->isAck)
    {
      spdlog::info("[VESC][{}<={}] COMM_SET_CUSTOM_CONFIG OK", this->id, src_id);
      return true;
    }
    return false;
  });
}


bool VESCDevice::send(VESC::Packet& pkt)
{
  return _host ? _host->send(nullptr, id, pkt) : false;
}

bool VESCDevice::sendRequest(VESC::Packet& pkt)
{
  pkt.isRequest = true;
  return _host ? _host->send(nullptr, id, pkt) : false;
}

bool VESCDevice::sendCmd(const std::string_view& cmd)
{
  spdlog::info("[VESC][{}]=> {}", id, cmd);
  vescpp::VESC::packets::TerminalCmd pkt(cmd);
  bool r = this->send(pkt);
  if(!r)
    spdlog::error("[VESC][{}] => {}, send error", id, cmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  return r;
}


bool VESCDevice::pktAddHandler(VESC::PktId pkt_id, pkt_handler_cb_t cb)
{
  if(_pkt_handlers.find(pkt_id) == _pkt_handlers.end())
  {
    //spdlog::debug("[VESCDevice][{}] Add Handler for Packet {}", id, pkt_id);
    _pkt_handlers[pkt_id] = cb;
    return true;
  }
  return false;
}

bool VESCDevice::pktProcess(Comm* comm, const VESC::BoardId src_id, std::unique_ptr<VESC::Packet>& pkt)
{
  if(_wait_for_pkt_id == pkt->id)
  {
    _wait_for_pkt_id = VESC::InvalidPktId;
    _promise.set_value(std::move(pkt));
    return true;
  }

  if(_pkt_handlers.find(pkt->id) != _pkt_handlers.end())
    return _pkt_handlers[pkt->id](comm, src_id, pkt);

  //Only output debug if this is a Device
  if(_host)
    spdlog::debug("[VESCDevice][{}<={}] Unhandled Packet {}", id, src_id, pkt->id);
  return false;
}

}