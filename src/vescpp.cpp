#include "vescpp/vescpp.hpp"
#include "vescpp/packet/vesc_packets.hpp"

namespace vescpp
{

VESCpp::VESCpp(VESC::BoardId this_id, Comm* comm)
: VESCDevice(this_id)
, _comm(comm)
{
  _fw.fw_version_major = 6;
  _fw.fw_version_minor = 5;
  _fw.hw_name = "Vescpp";
  for(auto& c: _fw.uuid) c = 0x00;
  _fw.uuid[11] = this_id;
  _fw.pairing_done = false;
  _fw.fw_test_version_number = 0x0;
  _fw.hw_type_vesc = ::VESC::HW_TYPE_CUSTOM_MODULE;
  _fw.custom_config = 0x00;
  _fw.is_valid = true;
  _comm->_vescpp[this_id] = this;

  //spdlog::debug("[{0}/0x{0:2X}] New VESCpp instance", _id);
  if(auto* _can = dynamic_cast<comm::CAN*>(comm))
  {
    _can->_can_handlers.emplace_back(((comm::CAN::Id)(::VESC::CAN_PACKET_PING<<8)|this_id), std::bind(&VESCpp::pingCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    //_can_handlers.emplace_back((::VESC::CAN_PACKET_PONG<<8)| _id, std::bind(&CAN::pongCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    _rx_pkts.resize(10);
    for(auto& pkt: _rx_pkts)
      pkt.state = PktState::Idle;

    _can->_can_handlers.emplace_back(((comm::CAN::Id)(::VESC::CAN_PACKET_PROCESS_SHORT_BUFFER<<8)|this_id), std::bind(&VESCpp::processShortBufferCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    _can->_can_handlers.emplace_back(((comm::CAN::Id)(::VESC::CAN_PACKET_FILL_RX_BUFFER<<8)|this_id)      , std::bind(&VESCpp::fillRXBufferCB,       this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    _can->_can_handlers.emplace_back(((comm::CAN::Id)(::VESC::CAN_PACKET_FILL_RX_BUFFER_LONG<<8)|this_id) , std::bind(&VESCpp::fillRXBufferCB,       this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    _can->_can_handlers.emplace_back(((comm::CAN::Id)(::VESC::CAN_PACKET_PROCESS_RX_BUFFER<<8)|this_id)   , std::bind(&VESCpp::processRXBufferCB,    this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
  }
}

bool send(const VESC::BoardId id, const VESC::Packet* pkt)
{
  return true;
}

bool VESCpp::add_peer(VESC::BoardId board_id, VESC::HwTypeId typ)
{
  //spdlog::debug("[{0}/0x{0:02X}] Add VESC Peer {1}: {2}", id, board_id, ::VESC::HW_TYPE_s(typ));

  switch(typ)
  {
    case ::VESC::HW_TYPE_VESC:
      _devs[board_id] = std::make_unique<VESCDrive>(board_id);
      break;
    case ::VESC::HW_TYPE_CUSTOM_MODULE:
      _devs[board_id] = std::make_unique<VESCCustomHw>(board_id);
      break;
    default:
      spdlog::error("[{0}/0x{0:02X}] Unsupported Peer type {1}/{2} for ID {3}", id, typ, ::VESC::HW_TYPE_s(typ), board_id);  
      return false;
  }
  VESC::packets::FwVersion pkt(true);
  _comm->send(id, board_id, pkt);

  return true; 
} 

void VESCpp::processShortBufferCB(comm::CAN* can, const comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len)
{
  //spdlog::debug("[{0}] Got CAN_PACKET_PROCESS_SHORT_BUFFER: {1:pn}", id, spdlog::to_hex(data, data+len));
  if(len < 2)
    return;

  const auto& tgt_id = id&0xFF;
  const auto& src_id = data[0];
  const auto& pkt_id = data[1];
  const auto& pkt_len = len-1;
  DataBuffer b; b.resize(pkt_len);
  for(size_t i=0;i<pkt_len;i++)
    b[i] = data[i+1];
  
  if(tgt_id != id)
    return;

  if(!processRawPacket(can, src_id, b, 0, pkt_len))
    spdlog::error(" Could not process Short Packet {}", pkt_id);
}

void VESCpp::fillRXBufferCB(comm::CAN* can, const comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len)
{
  const bool isLong = ((can_id&0xFF00)>>8) == ::VESC::CAN_PACKET_FILL_RX_BUFFER_LONG;
  const auto& header_len = (isLong ? 2 : 1);
  //spdlog::debug("[{0}] Got {1}      : {2:pn}", id, isLong ? "CAN_PACKET_FILL_RX_BUFFER_LONG" : "CAN_PACKET_FILL_RX_BUFFER", spdlog::to_hex(data, data+len));
  uint16_t rx_idx = (isLong ? (data[1]<<8) : 0x0000)| data[0];
  auto find_pkt = [this](PktState st, uint16_t rx_idx) -> size_t
  {
    size_t i = 0;
    for(const auto& pkt: _rx_pkts)
    {
      if(pkt.state == st && pkt.index == rx_idx)
        return i;
      i++;
    }
    return i;
  };

  //spdlog::debug("  Rxed index: {}", rx_idx);
  auto pkt_idx = find_pkt(rx_idx == 0 ? PktState::Idle : PktState::Streaming, rx_idx);
  //spdlog::debug("  ==> Pkt  index: {}", pkt_idx);
  if(pkt_idx >= _rx_pkts.size())
  {
    return;
  }
  auto& pkt = _rx_pkts[pkt_idx];
  //if(rx_idx == 0x00)
  //  spdlog::debug("  Start buffering Packet {}", pkt_idx);
  //else
  //  spdlog::debug("  Buffering Packet {}, from {} to {}", pkt_idx, pkt.index, pkt.index+len-header_len);
  pkt.last_t = Time::now();
  pkt.state = PktState::Streaming;
  
  for(size_t i=header_len;i<len;i++)
    pkt.buffer.push_back(data[i]);
  pkt.index += len-header_len;
}

void VESCpp::processRXBufferCB(comm::CAN* can, const comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len)
{
  //spdlog::debug("[{0}] Got CAN_PACKET_PROCESS_RX_BUFFER   : {1:pn}", id, spdlog::to_hex(data, data+len));
  //auto board_id = data[0];
  auto tgt_id   = can_id&0xFF;
  auto src_id   = data[0];
  auto cmd_send = data[1];
  auto rx_len   = ((uint16_t)(data[2]) << 8) | data[3];
  auto rx_crc   = ((uint16_t)(data[4]) << 8) | data[5];
  auto find_pkt = [this](PktState st, uint16_t rx_idx) -> size_t
  {
    size_t i = 0;
    for(const auto& pkt: _rx_pkts)
    {
      if(pkt.state == st && pkt.index == rx_idx)
        return i;
      i++;
    }
    return i;
  };

  auto pkt_idx = find_pkt(PktState::Streaming, rx_len);
  //spdlog::debug("  ==> Pkt  index: {}", pkt_idx);
  if(pkt_idx >= _rx_pkts.size())
  {
    spdlog::debug(" Can't process Packet with length: {}", rx_len);
    return;
  }
  auto& pkt = _rx_pkts[pkt_idx];
  auto pkt_len = pkt.buffer.size();
  auto pkt_crc = ::VESC::crc16(pkt.buffer);
  if(pkt_crc == rx_crc)
  {
    //spdlog::debug("  Process Packet {} with from {}, for {} cmd_send {}, length {}/{}, crc 0x{:04X}/0x{:04X}", pkt_idx, src_id, tgt_id, cmd_send, pkt_len, rx_len, pkt_crc, rx_crc);
    //spdlog::debug("    => {:np}", spdlog::to_hex(pkt.buffer));  

    switch(cmd_send)
    {
      case 0:
        break;
      case 1:
        if(tgt_id == id)
        {
          if(!processRawPacket(can, src_id, pkt.buffer, 0, pkt_len))
            spdlog::error(" Could not process Packet {}", pkt_idx);
        }
        break;
      case 2:
        //vesc_tool: //commands_process_packet(rx_buffer, rxbuf_len, 0);
        break;
      default:
        spdlog::error("  Unknown cmd_send value. Drop Packet {}", pkt_idx);
        break;
    }
  } 
  else
  {
    spdlog::error("  CRC mismatch. Drop Packet {} with length {}/{}, crc 0x{:04X}/0x{:04X}", pkt_idx, pkt_len, rx_len, pkt_crc, rx_crc);
  }

  // Reset Buffer 
  pkt.buffer.clear();
  pkt.index = 0x0000;
  pkt.state = PktState::Idle;
}

void VESCpp::pingCB(comm::CAN* can, const comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len)
{
  //spdlog::debug("[{0}] Got CAN_PACKET_PING                : {1:pn}", _port, spdlog::to_hex(data, data+len));
  if(len < 1)
    return;

  const auto& tgt_id = id&0xFF;
  const auto& src_id = data[0];

  uint8_t tx_data[2] = { id, fw.hw_type_vesc};
  can->write((::VESC::CAN_PACKET_PONG<<8)|src_id, tx_data, 2);
}

}