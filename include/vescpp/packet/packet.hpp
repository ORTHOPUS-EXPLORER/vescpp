#pragma once
#include "vescpp/common.hpp"

#include <nlohmann/json.hpp>

namespace vescpp::VESC
{
  class Packet
  {
    public:
      Packet(PktId id, size_t len_min, size_t len_max=0);

      bool encode(DataBuffer& buf, size_t start=0, size_t max_len=0);
      bool decode(const DataBuffer& buf, size_t start=0, size_t len=0);

      virtual bool encode_payload(DataBuffer& buf, size_t start=0, size_t len=0) = 0;
      virtual bool decode_payload(const DataBuffer& buf, size_t start=0, size_t len=0) = 0;

      virtual void setupTest() {}
      virtual nlohmann::json toJson() const;
      virtual bool fromJson(const nlohmann::json& json);

      const PktId id;
      bool isRequest;
      bool isAck;
      bool isValid;
      size_t _header_length, _payload_length;
      const size_t _payload_length_min,
                   _payload_length_max;
  };

  class RawPacket
    : public Packet
  {
  public:
    RawPacket(PktId id, size_t len_min, size_t len_max=0);
    RawPacket(PktId id, const DataBuffer& data);
    RawPacket(PktId id, const DataBuffer& data, size_t start, size_t len);

    bool encode_payload(DataBuffer& buf, size_t start=0, size_t max_len=0) override;
    bool decode_payload(const DataBuffer& buf, size_t start=0, size_t len=0) override;

    DataBuffer  data;
  };
}