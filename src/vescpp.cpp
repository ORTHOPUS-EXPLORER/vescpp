#include "vescpp/vescpp.hpp"
#include "vescpp/packet/vesc_packets.hpp"

namespace vescpp
{

bool VESCDevice::decodePacket(Comm* comm, const VESC::BoardId src_id, const DataBuffer& buff, size_t start, size_t len)
{
  const auto pkt_id = buff[start];
  switch(pkt_id)
  {
    case ::VESC::COMM_FW_VERSION:
      //spdlog::debug("[VESC][{}] Decoding FwVersion", id);
      return _fw.decode(buff, start, len);
    default:
      spdlog::debug("[{}<={}] Unhandled Packet {}", id, src_id, pkt_id);
  }
  return false;
}

VESCpp::VESCpp(VESC::BoardId this_id, Comm* comm, bool device_mode)
: VESCDevice(this_id)
, _comm(comm)
, _device_mode(device_mode)
{
  _fw.fw_version_major = 6;
  _fw.fw_version_minor = 2;
  _fw.hw_name = "VESCpp";
  _fw.uuid[ 0] = 0xBA; _fw.uuid[ 1] = 0xBA;
  _fw.uuid[ 2] = 0xD0; _fw.uuid[ 3] = 0x0D;
  _fw.uuid[ 4] = 0xBA; _fw.uuid[ 5] = 0xBA;
  _fw.uuid[ 6] = 0xF0; _fw.uuid[ 7] = 0x0D;
  _fw.uuid[ 8] = 0xBA; _fw.uuid[ 9] = 0xBA;
  _fw.uuid[10] = 0x00; _fw.uuid[11] = this_id;
  _fw.pairing_done = false;
  _fw.fw_test_version_number = 0x0;
  _fw.hw_type_vesc = ::VESC::HW_TYPE_CUSTOM_MODULE;
  _fw.custom_config = 0x00;
  _fw.is_valid = true;
  _comm->_vescpp[id] = this;

  spdlog::debug("[{}] New VESCpp instance, device_mode: {}", id, _device_mode);
  if(auto* _can = dynamic_cast<comm::CAN*>(comm))
  {
    if(_device_mode)
      _can->_can_handlers.emplace_back((comm::CAN::Id)(::VESC::CAN_PACKET_PING<<8)|id, std::bind(&VESCpp::pingCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    //else
    //  _can->_can_handlers.emplace_back((comm::CAN::Id)(::VESC::CAN_PACKET_PONG<<8)| id, std::bind(&VESCpp::pongCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

    _rx_pkts.resize(10);
    for(auto& pkt: _rx_pkts)
      pkt.state = PktState::Idle;

    _can->_can_handlers.emplace_back((comm::CAN::Id)(::VESC::CAN_PACKET_PROCESS_SHORT_BUFFER<<8)|id, std::bind(&VESCpp::processShortBufferCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    _can->_can_handlers.emplace_back((comm::CAN::Id)(::VESC::CAN_PACKET_FILL_RX_BUFFER<<8)|id      , std::bind(&VESCpp::fillRXBufferCB,       this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    _can->_can_handlers.emplace_back((comm::CAN::Id)(::VESC::CAN_PACKET_FILL_RX_BUFFER_LONG<<8|id) , std::bind(&VESCpp::fillRXBufferCB,       this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
    _can->_can_handlers.emplace_back((comm::CAN::Id)(::VESC::CAN_PACKET_PROCESS_RX_BUFFER<<8|this_id)   , std::bind(&VESCpp::processRXBufferCB,    this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
  }
}

bool VESCpp::send(Comm* comm, const VESC::BoardId tgt_id, VESC::Packet& pkt, uint8_t send_cmd)
{
  if(auto* can = dynamic_cast<comm::CAN*>(comm))
    return sendCAN(can, tgt_id, pkt, send_cmd);
  return false;
}

bool VESCpp::processRawPacket(Comm* comm, const VESC::BoardId src_id, const DataBuffer& buff, size_t start, size_t len)
{
  spdlog::trace("[{}<={}] Process raw packet", id, src_id, len);
  spdlog::trace("    => {:np}", spdlog::to_hex(buff));  
  if(len <= 2)
  {
    auto pkt_id = buff[start];
    auto payload = buff[start+1];

    DataBuffer outb;
    if(!_device_mode)
      return true;
    //spdlog::debug("Decoding request for Pkt Id: {}", pkt_id);
    switch(pkt_id)
    {
      case ::VESC::COMM_FW_VERSION:
         spdlog::debug("[{}=>{}] Reply to FW_VERSION request", id, src_id);
        return send(comm, src_id, _fw, 0x01);
      default:
        spdlog::warn("[{}<={}] Unhandled Request for Packet {}", id, src_id, pkt_id);
    }
    return false;
  }
  if(auto it = _devs.find(src_id); it != _devs.end())
    return it->second->decodePacket(comm, src_id, buff, start, len);
  return false;
}

bool VESCpp::add_peer(VESC::BoardId board_id, VESC::HwTypeId typ)
{
  spdlog::debug("[{}] Add VESC Peer {}: {}", id, board_id, ::VESC::HW_TYPE_s(typ));

  switch(typ)
  {
    case ::VESC::HW_TYPE_VESC:
      _devs[board_id] = std::make_unique<VESCDrive>(board_id);
      break;
    case ::VESC::HW_TYPE_CUSTOM_MODULE:
      _devs[board_id] = std::make_unique<VESCCustomHw>(board_id);
      break;
    default:
      spdlog::warn("[{}] Unsupported Peer type {}/{} for Peer {}", id, typ, ::VESC::HW_TYPE_s(typ), board_id);  
      _devs[board_id] = std::make_unique<VESCDevice>(board_id);
  }
  // SendRequest for FW_VERSION
  VESC::packets::FwVersion pkt(true);
  return send(_comm, board_id, pkt);
} 

bool VESCpp::sendCAN(comm::CAN* can, const VESC::BoardId tgt_id, VESC::Packet& pkt, uint8_t send_cmd)
{
  DataBuffer pktbuf, buf;
  if(!pkt.encode(pktbuf))
  {
    spdlog::error("[{}=>{}] Could not encode Packet {}", id, tgt_id, pkt.id);
    return false;
  }
  uint16_t sz = pktbuf.size();
  if(sz <= 6)
  {
    //spdlog::debug("[{}=>{}] Send CAN_PACKET_PROCESS_SHORT_BUFFER for {}", src_id, tgt_id, pkt.id);
    buf.push_back(id);   
    buf.push_back(0x00); // Process packet at receiver
    for(const auto& c: pktbuf)
      buf.push_back(c);
    return can->write((::VESC::CAN_PACKET_PROCESS_SHORT_BUFFER<<8)|tgt_id,buf.data(),buf.size());
  }
  else
  {
    uint16_t idx=0;
    auto pkt_crc = ::VESC::crc16(pktbuf,idx);
    spdlog::trace("[{}=>{}] Send CAN_PACKET_FILL_RX_BUFFER or CAN_PACKET_FILL_RX_BUFFER_LONG for Pkt {}, len {}, crc 0x{:04X}", id, tgt_id, pkt.id, sz, pkt_crc);
    spdlog::trace("    => {:np}", spdlog::to_hex(pktbuf));  
    while(idx<sz)
    {
      buf.clear();
      size_t max = sz-idx;
      if(idx < 255)
      {
        if(max > 7) max = 7;
      }
      else
      {
        buf.push_back(idx>>8);
        if(max > 6) max = 6;
      } 
      buf.push_back(idx&0xFF);


      for(size_t j=0;j<max;j++)
        buf.push_back(pktbuf[idx+j]);
      //spdlog::debug("  Chunk {}/{}, write {}", idx, sz, max);
      const auto can_id = ((idx<255 ? ::VESC::CAN_PACKET_FILL_RX_BUFFER : ::VESC::CAN_PACKET_FILL_RX_BUFFER_LONG)<<8)|tgt_id;
      if(!can->write(can_id,buf.data(),buf.size()))
      {
        spdlog::error("[{}=>{}] Pkt {} Could not write {}/{}, {} bytes", id, tgt_id, pkt.id, idx, sz, max);
        return false;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(10));
      idx += max;
    }
    spdlog::trace("[{}=>{}] Send CAN_PACKET_PROCESS_RX_BUFFER for Pkt {}, len {}/{}, crc 0x{:04X}, send_cmd {}", id, tgt_id, pkt.id, idx, sz, pkt_crc, send_cmd);
    buf.clear();
    buf.push_back(id);
    buf.push_back(send_cmd);
    buf.push_back(sz >> 8);
    buf.push_back(sz & 0xFF);
    buf.push_back(pkt_crc >> 8);
    buf.push_back(pkt_crc & 0xFF);
    return can->write((::VESC::CAN_PACKET_PROCESS_RX_BUFFER<<8)|tgt_id,buf.data(),buf.size());
  }
  
  return false;
}

std::vector<std::pair<VESC::BoardId, VESC::HwTypeId>> VESCpp::scanCAN(std::chrono::milliseconds timeout_ms)
{
  auto* can = dynamic_cast<comm::CAN*>(_comm);
  if(!can)
    return {};
  spdlog::debug("[{}] Scan CAN bus for VESC boards", id);
  std::vector<std::pair<VESC::BoardId, VESC::HwTypeId>> out;
  auto& hdlr = can->_can_handlers.emplace_back(((comm::CAN::Id)::VESC::CAN_PACKET_PONG<<8)|id, 
    [this, &out](comm::CAN* can, const comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len)
    {
      spdlog::trace("[{}] Got CAN PONG: {:pn}", id, spdlog::to_hex(data, data+len));
      out.emplace_back((VESC::BoardId)(data[0]), (VESC::HwTypeId)(data[1]));
    });
  for(uint8_t board_id=0;board_id<0xFF;board_id++)
  {
    uint8_t data = id&0xFF;
    spdlog::trace("[{}=>{}] Send PING", id, board_id);
    if(!can->write((::VESC::CAN_PACKET_PING<<8)| board_id, &data, 1))
    {
      spdlog::error("[{}=>{}] Error when sending PING", id, board_id);
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
  std::this_thread::sleep_for(timeout_ms);
  for (auto it=can->_can_handlers.begin(); it !=can->_can_handlers.end();++it)
  {
    if(hdlr.first == it->first && &hdlr.second == &it->second)
    {
      //spdlog::debug("Remove handler");
      can->_can_handlers.erase(it);
      --it;
    }
  }
  spdlog::debug("[{}] Done scanning CAN bus for VESC boards", id);
  return out;
}

void VESCpp::processShortBufferCB(comm::CAN* can, const comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len)
{
  spdlog::trace("[{}] Got CAN_PACKET_PROCESS_SHORT_BUFFER: {:pn}", id, spdlog::to_hex(data, data+len));
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
    spdlog::error("[{}<={}] Could not process Short Packet {}", id, src_id, tgt_id, pkt_id);
}

void VESCpp::fillRXBufferCB(comm::CAN* can, const comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len)
{
  const bool isLong = ((can_id&0xFF00)>>8) == ::VESC::CAN_PACKET_FILL_RX_BUFFER_LONG;
  const auto& header_len = (isLong ? 2 : 1);
  spdlog::trace("[{}] Got {}      : {:pn}", id, isLong ? "CAN_PACKET_FILL_RX_BUFFER_LONG" : "CAN_PACKET_FILL_RX_BUFFER", spdlog::to_hex(data, data+len));
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
  if(rx_idx == 0x00)
    spdlog::trace("[{}] Start buffering Packet {}", id, pkt_idx);
  else
    spdlog::trace("[{}] Buffering Packet {}, from {} to {}", id, pkt_idx, pkt.index, pkt.index+len-header_len);
  pkt.last_t = Time::now();
  pkt.state = PktState::Streaming;
  
  for(size_t i=header_len;i<len;i++)
    pkt.buffer.push_back(data[i]);
  pkt.index += len-header_len;
}

void VESCpp::processRXBufferCB(comm::CAN* can, const comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len)
{
  spdlog::trace("[{}] Got CAN_PACKET_PROCESS_RX_BUFFER   : {:pn}", id, spdlog::to_hex(data, data+len));
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
    spdlog::debug("[{}] Can't process Packet with length: {}", id, rx_len);
    return;
  }
  auto& pkt = _rx_pkts[pkt_idx];
  auto pkt_len = pkt.buffer.size();
  auto pkt_crc = ::VESC::crc16(pkt.buffer);
  if(pkt_crc == rx_crc)
  {
    spdlog::trace("[{}][{}<={}] Process Packet {}, length {}/{}, crc 0x{:04X}/0x{:04X}, cmd_send: {}", id, tgt_id, src_id, pkt_idx, pkt_len, rx_len, pkt_crc, rx_crc, cmd_send);
    spdlog::trace("    => {:np}", spdlog::to_hex(pkt.buffer));  

    switch(cmd_send)
    {
      case 0:
        break;
      case 1:
        if(tgt_id == id)
        {
          if(!processRawPacket(can, src_id, pkt.buffer, 0, pkt_len))
            spdlog::error("[{}<={}] Could not process Packet {}", id, src_id, pkt_idx);
        }
        break;
      case 2:
        //vesc_tool: //commands_process_packet(rx_buffer, rxbuf_len, 0);
        break;
      default:
        spdlog::error("[{}<={}] Unknown cmd_send value. Drop Packet {}", id, src_id, pkt_idx);
        break;
    }
  } 
  else
  {
    spdlog::error("[{}<={}] CRC mismatch. Drop Packet {}, length {}/{}, crc 0x{:04X}/0x{:04X}, cmd_send: {}", id, tgt_id, src_id, pkt_idx, pkt_len, rx_len, pkt_crc, rx_crc, cmd_send);
  }

  // Reset Buffer 
  pkt.buffer.clear();
  pkt.index = 0x0000;
  pkt.state = PktState::Idle;
}

void VESCpp::pongCB(comm::CAN* can, const comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len)
{
  spdlog::trace("[{}] Got CAN_PACKET_PONG                : {:pn}", id, spdlog::to_hex(data, data+len));
}

void VESCpp::pingCB(comm::CAN* can, const comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len)
{
  spdlog::trace("[{}] Got CAN_PACKET_PING                : {:pn}", id, spdlog::to_hex(data, data+len));
  const auto& src_id = len < 1 ? 0x00 : data[0];

  uint8_t tx_data[2] = { id, fw.hw_type_vesc};
  can->write((::VESC::CAN_PACKET_PONG<<8)|src_id, tx_data, 2);
}

}