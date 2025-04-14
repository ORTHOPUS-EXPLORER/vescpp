#include "vescpp/packet/vesc_packets.hpp"

using json = nlohmann::json;

namespace vescpp::VESC::packets
{

template<VESC::PktId ID>
ConfPkt<ID>::ConfPkt(const uint8_t mid)
: Packet(id(), 4, 0xFFFF)
, _mid(mid)
{
}

template<VESC::PktId ID>
json ConfPkt<ID>::toJson() const
{ 
    spdlog::trace("[ConfPkt<{}>::toJson]", ID);
    return {
        {"signature", signature},
        {"data", json::binary_t{payload}}
    }; 
}

template<VESC::PktId ID>
bool ConfPkt<ID>::fromJson(const json& json)
{
    spdlog::trace("[ConfPkt<{}>::fromJson]", ID);
    signature = json["signature"];
    auto& jdata = json["data"]["bytes"];
    auto sz = jdata.size();
    payload.resize(sz);
    //spdlog::debug("Sz: {}, Data: {}", sz, spdlog::to_hex(jdata));
    std::copy_n(jdata.begin(), sz, payload.begin());
    return true;
}

template<VESC::PktId ID>
bool ConfPkt<ID>::encode_payload(DataBuffer& buf, size_t index, size_t max_len)
{
    spdlog::trace("[ConfPkt<{}>::encode_payload] Start {}, MaxLen {}", ID, index, max_len);
    const auto psz = payload.size();
    const auto nlen = index+4+psz;
    if(max_len && max_len < nlen)
        return false;
    
    buf.resize(nlen);

    // Pack signature
    //spdlog::debug("Signature: {0:d}/0x{0:08X}", signature);
    for(size_t i=4;i>0;--i)
        buf[index++] = (uint8_t)((signature>>(8*(i-1)))&0xFF);

    // Copy raw payload
    //spdlog::debug("Payload: ", spdlog::to_hex(payload));
    for(const auto& v: payload)
        buf[index++] = v;
    
    //spdlog::debug(" ==> {:np}", spdlog::to_hex(buf));
    return true;
}

template<VESC::PktId ID>
bool ConfPkt<ID>::decode_payload(const DataBuffer& buf, size_t index, size_t len)
{
    spdlog::trace("[ConfPkt<{}>::decode_payload] Start {}, Len {}", ID, index, len);
    //spdlog::debug("  ==> {:np}", spdlog::to_hex(buf));

    // Exit if Paket is too short, minimum length is signature, which is 4 bytes
    if(len-index < 4)
        return false;

    // Unpack signature
    signature = 0;
    for(size_t i = 0;i<4;i++)
        signature = (signature<<8)| buf[index++];
    // Copy raw payload
    payload.resize(len-index+1);
    for(size_t i=0;index<len;index++,i++)
        payload[i] = buf[index];
    /* TODO: Load MC conf XML, exported from VESCTool
                and unpack this to a map [param_name, param_value]
                <std::string, std::variant<bool, int, float, std::string>> or something like that
                and then remove payload
    */
    //spdlog::debug("  ==> {:np}", spdlog::to_hex(payload));
    return true;
}

template class ConfPkt<::VESC::COMM_GET_MCCONF>;
template class ConfPkt<::VESC::COMM_SET_MCCONF>;
template class ConfPkt<::VESC::COMM_GET_APPCONF>;
template class ConfPkt<::VESC::COMM_SET_APPCONF>;
template class ConfPkt<::VESC::COMM_GET_CUSTOM_CONFIG>;
template class ConfPkt<::VESC::COMM_SET_CUSTOM_CONFIG>;

}