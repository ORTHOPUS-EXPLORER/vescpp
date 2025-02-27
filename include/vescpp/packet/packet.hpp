#pragma once
#include "vescpp/common.hpp"

namespace vescpp::VESC
{
  class Packet
  {
    public:
      Packet(VESC::PktId id, bool isReq, size_t len_min, size_t len_max=0)
        : id(id)
        , _isRequest(isReq)
        , is_valid(false)
        , _payload_length(0)
        , _payload_length_min(len_min)
        , _payload_length_max(len_max == 0 ? len_min : len_max)
      {
      }

      bool encode(DataBuffer& buf, size_t start=0, size_t max_len=0)
      {
        buf.push_back(id);
        _header_length = 1;
        if(!_isRequest)
          return encode_payload(buf, start, max_len);
        return true;
      }

      bool decode(const DataBuffer& buf, size_t start=0, size_t len=0)
      {
        if(!len)
          len = buf.size() - start;
        if(len < 1 || id != buf[0])
        {
          spdlog::error("[Packet] Can't decode packet, len {}. Id {}/{}", len, id, buf[0]);
          return false;
        }
        return decode_payload(buf, start+1, len-1);
      }

      virtual bool encode_payload(DataBuffer& buf, size_t start=0, size_t len=0) = 0;
      virtual bool decode_payload(const DataBuffer& buf, size_t start=0, size_t len=0) = 0;

      virtual void setupTest() {}

      const VESC::PktId id;
      bool _isRequest;
      bool is_valid;
      size_t _header_length, _payload_length;
      const size_t _payload_length_min,
                   _payload_length_max;
  };
}