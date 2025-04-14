#include "vescpp/packet/packet.hpp"

namespace vescpp::VESC
{

Packet::Packet(PktId id, size_t len_min, size_t len_max)
  : id(id)
  , isRequest(false)
  , isAck(false)
  , isValid(false)
  , _payload_length(0)
  , _payload_length_min(len_min)
  , _payload_length_max(len_max == 0 ? len_min : len_max)
{
}

nlohmann::json Packet::toJson() const
{
  spdlog::error("[Packet::toJson][{}] Unhandled conversion to JSON", id);
  return {};
}

bool Packet::fromJson(const nlohmann::json& json)
{
  spdlog::error("[Packet::fromJson][{}] Unhandled conversion from JSON", id);
  return false;
}

bool Packet::encode(DataBuffer& buf, size_t start, size_t max_len)
{
  buf.push_back(id);
  _header_length = 1;
  if(!isRequest)
    return encode_payload(buf, start+_header_length, max_len);
  spdlog::trace("[Packet][{}] isRequest", id);
  return true;
}

bool Packet::decode(const DataBuffer& buf, size_t start, size_t len)
{
  if(!len)
    len = buf.size() - start;
  if(len < 1 || id != buf[0])
  {
    spdlog::error("[Packet] Can't decode packet, len {}. Id {}/{}", len, id, buf[0]);
    return false;
  }
  if(len == 1)
  {
    isAck = true;
    spdlog::trace("[Packet][{}] isAck", id);
    return true;
  }
  return decode_payload(buf, start+1, len-1);
}

}