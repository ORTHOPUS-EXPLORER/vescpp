#ifndef COMMON_MAIN
    #define COMMON_MAIN
#endif
#include "vescpp_tests.hpp"
#include "vescpp/packet/vesc_packets.hpp"


TEST_CASE("Test Packet Factory","[vescpp][VESC][Packets]")
{
  spdlog::cfg::load_env_levels();

  using namespace vescpp::VESC;
  REQUIRE(packets::init());
  {
    auto pkt = packets::create(::VESC::COMM_FW_VERSION);
    auto pkt2 = packets::create(::VESC::COMM_FW_VERSION);
    REQUIRE(pkt != nullptr);
    REQUIRE(pkt2 != nullptr);
    REQUIRE(dynamic_cast<packets::FwVersion*>(pkt.get()) != nullptr);
    REQUIRE(pkt != pkt2);
  }
  {
    auto pkt = packets::create(::VESC::COMM_FW_VERSION);
    REQUIRE(pkt != nullptr);
    REQUIRE(dynamic_cast<packets::FwVersion*>(pkt.get()) != nullptr);
  }

}

TEST_CASE("Packet FwVersion with payload","[vescpp][VESC][Packets]")
{
  spdlog::cfg::load_env_levels();

  vescpp::DataBuffer buf;

  vescpp::VESC::packets::FwVersion pkt, pkt2;
  pkt.setupTest();
  REQUIRE(pkt.encode(buf));
  auto sz = buf.size()-pkt._header_length;
  REQUIRE(sz >= pkt._payload_length_min);
  REQUIRE(sz == pkt._payload_length);
  REQUIRE(sz <= pkt._payload_length_max);
  spdlog::debug("Encoded FwVersion:{:np}",spdlog::to_hex(buf));
  REQUIRE(pkt2.decode(buf));
  REQUIRE(pkt.hw_name.length() == pkt2.hw_name.length());
  REQUIRE(pkt.hw_name == pkt2.hw_name);
  spdlog::debug("Decoded FwVersion: Name '{}', UUID '{:np}'", pkt2.hw_name, spdlog::to_hex(pkt2.uuid));
}

TEST_CASE("Packet FwVersion request","[vescpp][VESC][Packets]")
{
  spdlog::cfg::load_env_levels();

  vescpp::DataBuffer buf;

  vescpp::VESC::packets::FwVersion pkt; 
  pkt.isRequest = true;
  pkt.setupTest();
  REQUIRE(pkt.encode(buf));
  auto sz = buf.size();
  REQUIRE(sz == pkt._header_length);
  spdlog::debug("Encoded FwVersion request:{:np}",spdlog::to_hex(buf));
}

TEST_CASE("Packet print","[vescpp][VESC][Packets]")
{
  spdlog::cfg::load_env_levels();

  vescpp::DataBuffer buf;
  const std::string s = "Hello";
  const auto ssz = s.length();

  vescpp::VESC::packets::Print pkt(s),pkt2;
  REQUIRE(pkt.encode(buf));
  auto sz = buf.size()-pkt._header_length;
  REQUIRE(sz >= pkt._payload_length_min);
  REQUIRE(sz == pkt._payload_length);
  REQUIRE(sz <= pkt._payload_length_max);
  spdlog::debug("Encoded Print:{:np}",spdlog::to_hex(buf));
  REQUIRE(pkt2.decode(buf));
  REQUIRE(pkt.str.length() == pkt2.str.length());
  REQUIRE(pkt.str == pkt2.str);
  spdlog::debug("Decoded Print: '{}'", pkt2.str);
}

TEST_CASE("Terminal CMD print","[vescpp][VESC][Packets]")
{
  spdlog::cfg::load_env_levels();

  vescpp::DataBuffer buf;
  const std::string s = "Hello";
  const auto ssz = s.length();

  vescpp::VESC::packets::TerminalCmd pkt(s),pkt2;
  REQUIRE(pkt.encode(buf));
  auto sz = buf.size()-pkt._header_length;
  REQUIRE(sz >= pkt._payload_length_min);
  REQUIRE(sz == pkt._payload_length);
  REQUIRE(sz <= pkt._payload_length_max);
  spdlog::debug("Encoded Command:{:np}",spdlog::to_hex(buf));
  REQUIRE(pkt2.decode(buf));
  REQUIRE(pkt.str.length() == pkt2.str.length());
  REQUIRE(pkt.str == pkt2.str);
  spdlog::debug("Decoded Command: '{}'", pkt2.str);
}