#pragma once
#include <string>

#include "vescpp/packet/packet.hpp"
namespace vescpp::VESC::packets
{

bool init();
std::unique_ptr<Packet> create(PktId);
std::unique_ptr<Packet> create(PktId, const DataBuffer& buf, size_t start=0, size_t len=0);

class FwVersion
: public Packet
{
public:
    FwVersion();
    bool encode_payload(DataBuffer& buf, size_t start=0, size_t max_len=0) override;
    bool decode_payload(const DataBuffer& buf, size_t start=0, size_t len=0) override;
    void setupTest() override;

    nlohmann::json toJson() const override;
    bool fromJson(const nlohmann::json& json) override;

    static constexpr PktId id() { return ::VESC::COMM_FW_VERSION; }
    static inline Packet* create() { return new FwVersion; };

    uint8_t fw_version_major;
    uint8_t fw_version_minor;
    std::string hw_name;
    std::array<uint8_t, 12> uuid;
    bool pairing_done;
    uint8_t fw_test_version_number;
    uint8_t hw_type_vesc;
    uint8_t custom_config;
};

class Print
: public Packet
{
public:
    Print(const std::string_view& s="");
    bool encode_payload(DataBuffer& buf, size_t start=0, size_t max_len=0) override;
    bool decode_payload(const DataBuffer& buf, size_t start=0, size_t len=0) override;

    static constexpr PktId id() { return ::VESC::COMM_PRINT; }
    static Packet* create() { return new Print(); };

    std::string str;
};

class TerminalCmd
: public Packet
{
public:
    TerminalCmd(const std::string_view& s="");
    bool encode_payload(DataBuffer& buf, size_t start=0, size_t max_len=0) override;
    bool decode_payload(const DataBuffer& buf, size_t start=0, size_t len=0) override;

    static constexpr PktId id() { return ::VESC::COMM_TERMINAL_CMD; }
    static Packet* create() { return new TerminalCmd(); };

    std::string str;
};

template<VESC::PktId ID>
class ConfPkt
: public Packet
{
public:
    ConfPkt(const uint8_t mid=0);

    nlohmann::json toJson() const override;
    bool fromJson(const nlohmann::json& json) override;

    bool encode_payload(DataBuffer& buf, size_t index=0, size_t max_len=0) override;
    bool decode_payload(const DataBuffer& buf, size_t index=0, size_t len=0) override;

    static constexpr PktId id() { return ID; }
    static Packet* create() { return new ConfPkt<ID>(); };

    uint8_t _mid;
    uint32_t signature;
    DataBuffer payload;
};

using GetMCConf     = ConfPkt<::VESC::COMM_GET_MCCONF>;
extern template class ConfPkt<::VESC::COMM_GET_MCCONF>;
using SetMCConf     = ConfPkt<::VESC::COMM_SET_MCCONF>;
extern template class ConfPkt<::VESC::COMM_SET_MCCONF>;

using GetAppConf    = ConfPkt<::VESC::COMM_GET_APPCONF>;
extern template class ConfPkt<::VESC::COMM_GET_APPCONF>;
using SetAppConf    = ConfPkt<::VESC::COMM_SET_APPCONF>;
extern template class ConfPkt<::VESC::COMM_SET_APPCONF>;

using GetCustomConf = ConfPkt<::VESC::COMM_GET_CUSTOM_CONFIG>;
extern template class ConfPkt<::VESC::COMM_GET_CUSTOM_CONFIG>;
using SetCustomConf = ConfPkt<::VESC::COMM_SET_CUSTOM_CONFIG>;
extern template class ConfPkt<::VESC::COMM_SET_CUSTOM_CONFIG>;

}

