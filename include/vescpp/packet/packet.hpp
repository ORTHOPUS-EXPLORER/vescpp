#pragma once
#include "vescpp/common.hpp"

namespace vescpp::VESC
{
  class Packet
  {
    public:
      Packet(VESC::PktId id, bool isReq, size_t len_min, size_t len_max=0)
        : _id(id)
        , _isRequest(isReq)
        , _payload_length(0)
        , _payload_length_min(len_min)
        , _payload_length_max(len_max == 0 ? len_min : len_max)
      {
      }

      bool encode(DataBuffer& buf)
      {
        buf.push_back(_id);
        _header_length = 1;
        if(!_isRequest)
          return encode_payload(buf);
        return true;
      }

      bool decode(const DataBuffer& buf)
      {
        if(_id != buf[0])
          return false;
        return decode_payload(buf, 1);
      }

      virtual bool encode_payload(DataBuffer& buf) = 0;
      virtual bool decode_payload(const DataBuffer& buf, size_t start=0, size_t len=0) = 0;

      virtual void setupTest() {}

      const VESC::PktId _id;
      bool _isRequest;
      size_t _header_length, _payload_length;
      const size_t _payload_length_min,
                   _payload_length_max;
  };
}