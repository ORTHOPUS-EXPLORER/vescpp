#include "vescpp/packet/vesc_packets.hpp"

namespace vescpp::VESC::packets
{

Print::Print(const std::string_view& s)
    : Packet(id(), 0, 0xFFFF)
    , str(s)
{
}

bool Print::encode_payload(DataBuffer& buf, size_t start, size_t max_len)
{
    size_t str_length = str.length()+1;
    for(const auto& c: str)
        buf.push_back(c);
    buf.push_back('\0');
    _payload_length = str_length;
    return true;
}

bool Print::decode_payload(const DataBuffer& buf, size_t start, size_t len)
{
    if(len == 0) 
        len = buf.size()-start;
    auto str_len = len - _payload_length_min;
    size_t idx=start;
    if(len < _payload_length_min)
    {
        spdlog::error("[Print] Can't decode payload, not enough data: {}/{}", len, _payload_length_min);
        return false;
    }
    str.resize(str_len);
    for(size_t i=0;i<str_len;i++)
    {
        str[i] = buf[idx++];
        if(str[i] == '\0')
        {
            str_len = i;
            break;
        }
    }
    str.resize(str_len);
    return true;
}

TerminalCmd::TerminalCmd(const std::string_view& s)
    : Packet(id(), 0, 0xFFFF)
    , str(s)
{
}

bool TerminalCmd::encode_payload(DataBuffer& buf, size_t start, size_t max_len)
{
    size_t str_length = str.length()+1;
    for(const auto& c: str)
        buf.push_back(c);
    buf.push_back('\0');
    _payload_length = str_length;
    return true;
}

bool TerminalCmd::decode_payload(const DataBuffer& buf, size_t start, size_t len)
{
    if(len == 0) 
        len = buf.size()-start;
    auto str_len = len - _payload_length_min;
    size_t idx=start;
    if(len < _payload_length_min)
    {
        spdlog::error("[Print] Can't decode payload, not enough data: {}/{}", len, _payload_length_min);
        return false;
    }
    str.resize(str_len);
    for(size_t i=0;i<str_len;i++)
    {
        str[i] = buf[idx++];
        if(str[i] == '\0')
        {
            str_len = i;
            break;
        }
    }
    str.resize(str_len);
    return true;
}


}