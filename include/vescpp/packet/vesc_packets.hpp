#pragma once
#include "vescpp/packet/packet.hpp"

namespace vescpp::VESC::packets
{

class FwVersion
: public Packet
{
public:
    FwVersion(bool isreq=false);
    bool encode_payload(DataBuffer& buf, size_t start=0, size_t max_len=0) override;
    bool decode_payload(const DataBuffer& buf, size_t start=0, size_t len=0) override;
    void setupTest() override;

    uint8_t fw_version_major;
    uint8_t fw_version_minor;
    std::string hw_name;
    std::array<uint8_t, 12> uuid;
    bool pairing_done;
    uint8_t fw_test_version_number;
    uint8_t hw_type_vesc;
    uint8_t custom_config;
};

}