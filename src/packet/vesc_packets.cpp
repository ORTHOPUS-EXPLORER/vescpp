#include "vescpp/packet/packet.hpp"
#include "vescpp/packet/vesc_packets.hpp"
#include <map>

namespace vescpp::VESC::packets
{

static std::map<PktId, vescpp::VESC::Packet*(*)(void)> _knownPkts;

template<typename T>
constexpr bool registerPacketType(void)
{
  _knownPkts[T::id()] = &T::create;
  return true;
}

bool init()
{
  bool b = true;
  b = b && registerPacketType<FwVersion>();
  b = b && registerPacketType<Print>();
  b = b && registerPacketType<TerminalCmd>();
  b = b && registerPacketType<GetMCConf>();
  b = b && registerPacketType<SetMCConf>();
  b = b && registerPacketType<GetAppConf>();
  b = b && registerPacketType<SetAppConf>();
  b = b && registerPacketType<GetCustomConf>();
  b = b && registerPacketType<SetCustomConf>();
  return b;
}

std::unique_ptr<Packet> create(PktId id)
{
  if(auto it=_knownPkts.find(id); it != _knownPkts.end())
    return std::unique_ptr<Packet>{it->second()};
  return nullptr;
}

std::unique_ptr<Packet> create(PktId id, const DataBuffer& buf, size_t start, size_t len)
{
  if(auto pkt = create(id))
  {
    if(pkt->decode(buf, start, len))
      return pkt;
    spdlog::debug("[packets::create] Could not decode Packet with ID: {}", id);
    return nullptr;
  }
  spdlog::debug("[packets::create] Unkown Packet Type with ID: {}", id);
  return nullptr;
}

}