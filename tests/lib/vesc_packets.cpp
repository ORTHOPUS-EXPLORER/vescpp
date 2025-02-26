#ifndef COMMON_MAIN
    #define COMMON_MAIN
#endif
#include "vescpp_tests.hpp"
#include "vescpp/packet/vesc_packets.hpp"

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
  REQUIRE(pkt.hw_name == pkt2.hw_name);
  spdlog::debug("Decoded FwVersion: Name '{}', UUID '{:np}'", pkt2.hw_name, spdlog::to_hex(pkt2.uuid));
}

TEST_CASE("Packet FwVersion request","[vescpp][VESC][Packets]")
{
  spdlog::cfg::load_env_levels();

  vescpp::DataBuffer buf;

  vescpp::VESC::packets::FwVersion pkt(true);
  pkt.setupTest();
  REQUIRE(pkt.encode(buf));
  auto sz = buf.size();
  REQUIRE(sz == pkt._header_length);
  spdlog::debug("Encoded FwVersion:{:np}",spdlog::to_hex(buf));
}