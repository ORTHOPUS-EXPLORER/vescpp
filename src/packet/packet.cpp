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
  spdlog::error("[Packet::toJson][{:d}] Unhandled conversion to JSON", id);
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

RawPacket::RawPacket(PktId id, size_t len_min, size_t len_max)
  : Packet(id, len_min, len_max)
{
  data.reserve(len_max ? len_max : len_min);
}

RawPacket::RawPacket(PktId id, const DataBuffer& vec)
  : Packet(id, vec.size(), 0)
  , data(vec)
{

}

RawPacket::RawPacket(PktId id, const DataBuffer& vec, size_t start, size_t len)
  : Packet(id, vec.size(), 0)
{
  data.resize(len);
  for(size_t i=0;i<len;i++)
    data[i] = vec[start+i];
}

bool RawPacket::encode_payload(DataBuffer& buf, size_t start, size_t max_len)
{
  for(const auto& b: data)
    buf.push_back(b);
    _payload_length = data.size();
        
  return true;
}

bool RawPacket::decode_payload(const DataBuffer& buf, size_t start, size_t len)
{
  if(len < _payload_length_min)
  {
      spdlog::error("[RawPacket] Can't decode payload, not enough data: {}/{}", len, _payload_length_min);
      return false;
  }
  data.resize(len);
  for(size_t i=0;i<len;i++)
    data[i] = buf[start+i];
  return true;
}

}