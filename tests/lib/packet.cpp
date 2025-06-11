#include "vescpp_tests.hpp"
#include "vescpp/packet/packet.hpp"

TEST_CASE("test true","[vescpp][lib][packet]")
{
  REQUIRE(true);
}

TEST_CASE("Empty RawPacket","[vescpp][lib][packet]")
{
  spdlog::cfg::load_env_levels();

  vescpp::DataBuffer buf;
  const auto id = ::VESC::COMM_PING_CAN;

  vescpp::VESC::RawPacket pkt(id, {});
    // Encode
  REQUIRE(pkt.encode(buf));
  auto sz = buf.size();
  REQUIRE(sz == 1);
  REQUIRE(buf[0] == (uint8_t)id);
  spdlog::debug("Encoded RawPacket:{:np}",spdlog::to_hex(buf));
  // FIXME: Test all cases when decoding !
}

TEST_CASE("Normal RawPacket","[vescpp][lib][packet]")
{
  spdlog::cfg::load_env_levels();

  vescpp::DataBuffer buf;

  const auto id = ::VESC::COMM_PING_CAN;
  const vescpp::DataBuffer payload = {0x12,0x34,0x56,0x78};
  const auto psz = payload.size();

  vescpp::VESC::RawPacket pkt(id, payload);
    // Encode
  REQUIRE(pkt.encode(buf));
  auto sz = buf.size();
  REQUIRE(sz == (psz + 1));
  REQUIRE(buf[0] == (uint8_t)id);
  for(size_t i=1;i<sz;i++)
    REQUIRE(buf[i] == payload[i-1]);
  spdlog::debug("Encoded RawPacket:{:np}",spdlog::to_hex(buf));
    // Decode full packet with lower min_len
  vescpp::VESC::RawPacket pkt2(id, psz-1);
  REQUIRE(pkt2.decode(buf));
  REQUIRE(pkt2.id == id);
  REQUIRE(pkt2.data.size() == psz);
  spdlog::debug("Decoded RawPacket ID 0x{:02x}, payload:{:np}",pkt2.id, spdlog::to_hex(pkt2.data));
    // FIXME: Test all cases when decoding !
}