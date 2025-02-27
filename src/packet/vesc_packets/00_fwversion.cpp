#include "vescpp/packet/vesc_packets.hpp"

namespace vescpp::VESC::packets
{

FwVersion::FwVersion(bool isreq)
    : Packet(::VESC::COMM_FW_VERSION, isreq, 18, 30)
{}

bool FwVersion::encode_payload(DataBuffer& buf, size_t start, size_t max_len)
{
    buf.push_back(fw_version_major);
    buf.push_back(fw_version_minor);
    size_t name_length = hw_name.length()+1;
    for(const auto& c: hw_name)
        buf.push_back(c);
    buf.push_back('\0');
    for(const auto& c: uuid)
        buf.push_back(c);
    buf.push_back(pairing_done ? 0x01 : 0x00);
    buf.push_back(fw_test_version_number);
    buf.push_back(hw_type_vesc);
    buf.push_back(custom_config);
    _payload_length = 18+name_length;
    return true;
}

bool FwVersion::decode_payload(const DataBuffer& buf, size_t start, size_t len)
{
    if(len == 0) 
        len = buf.size()-start;
    auto name_len = len - _payload_length_min;
    size_t idx=start;
    if(len < _payload_length_min)
    {
        spdlog::error("[FwVersion] Can't decode payload, not enough data: {}/{}", len, _payload_length_min);
        return false;
    }

    fw_version_major = buf[idx++];
    fw_version_minor = buf[idx++];
    hw_name.resize(name_len);
    for(size_t i=0;i<name_len;i++)
    {
        hw_name[i] = buf[idx++];
        if(hw_name[i] == '\0')
        {
            name_len = i;
            break;
        }
    }
    for(auto& c: uuid)
        c = buf[idx++];
    pairing_done = buf[idx++] == 0x01 ? true : false;
    fw_test_version_number = buf[idx++];
    hw_type_vesc = buf[idx++];
    custom_config = buf[idx++];
    is_valid = true;
    return true;
}

void FwVersion::setupTest()
{
    fw_version_major = 6;
    fw_version_minor = 5;
    hw_name = "TestDevice";
    size_t i=0x10; for(auto& c: uuid) c = i++;
    pairing_done = false;
    fw_test_version_number = 0x1;
    hw_type_vesc = ::VESC::HW_TYPE_CUSTOM_MODULE;
    custom_config = 0x00;
    is_valid = true;
}

}