#include "vescpp/vescpp.hpp"
#include "vescpp/packet/vesc_packets.hpp"

using namespace std::chrono_literals;

namespace vescpp
{

VESCpp::VESCpp(VESC::BoardId this_id, Comm* comm, bool device_mode)
: VESCDevice(this_id, nullptr, false)
, _comm(comm)
, _device_mode(device_mode)
, _last_can_t()
, _last_pkt_t()
{
  VESC::packets::init();

  _fw = std::make_shared<VESC::packets::FwVersion>();
  _fw->fw_version_major = 6;
  _fw->fw_version_minor = 2;
  _fw->hw_name = "VESCpp";
  _fw->uuid[ 0] = 0xBA; _fw->uuid[ 1] = 0xBA;
  _fw->uuid[ 2] = 0xD0; _fw->uuid[ 3] = 0x0D;
  _fw->uuid[ 4] = 0xBA; _fw->uuid[ 5] = 0xBA;
  _fw->uuid[ 6] = 0xF0; _fw->uuid[ 7] = 0x0D;
  _fw->uuid[ 8] = 0xBA; _fw->uuid[ 9] = 0xBA;
  _fw->uuid[10] = 0x00; _fw->uuid[11] = this_id;
  _fw->pairing_done = false;
  _fw->fw_test_version_number = 0x0;
  _fw->hw_type_vesc = ::VESC::HW_TYPE_CUSTOM_MODULE;
  _fw->custom_config = 0x00;

  _comm->_vescpp[id] = this; // FIXME: Figure out if we keep this or not
  
  spdlog::debug("[{}] New VESCpp instance, device_mode: {}", id, _device_mode);
  if(auto* _can = dynamic_cast<comm::CAN*>(comm))
  {
    using namespace std::placeholders;

    if(_device_mode)
      _can->_can_handlers.emplace_back((comm::CAN::Id)(::VESC::CAN_PACKET_PING<<8)|id, std::bind(&VESCpp::pingCB, this, _1, _2, _3, _4));
    //else
    //  _can->_can_handlers.emplace_back((comm::CAN::Id)(::VESC::CAN_PACKET_PONG<<8)| id, std::bind(&VESCpp::pongCB, this, _1, _2, _3, _4));

    _rx_pkts.resize(10);
    for(auto& pkt: _rx_pkts)
      pkt.state = PktState::Idle;
    
    _can->_can_handlers.emplace_back((comm::CAN::Id)(::VESC::CAN_PACKET_PROCESS_SHORT_BUFFER<<8)|id   , std::bind(&VESCpp::processShortBufferCB, this, _1, _2, _3, _4));
    _can->_can_handlers.emplace_back((comm::CAN::Id)(::VESC::CAN_PACKET_FILL_RX_BUFFER<<8)|id         , std::bind(&VESCpp::fillRXBufferCB,       this, _1, _2, _3, _4));
    _can->_can_handlers.emplace_back((comm::CAN::Id)(::VESC::CAN_PACKET_FILL_RX_BUFFER_LONG<<8|id)    , std::bind(&VESCpp::fillRXBufferCB,       this, _1, _2, _3, _4));
    _can->_can_handlers.emplace_back((comm::CAN::Id)(::VESC::CAN_PACKET_PROCESS_RX_BUFFER<<8|this_id) , std::bind(&VESCpp::processRXBufferCB,    this, _1, _2, _3, _4));
  }
  if(!device_mode)
  {
    /*pktAddHandler(::VESC::COMM_SET_MCCONF,[this](Comm* comm, const VESC::BoardId src_id, std::shared_ptr<VESC::Packet>& pkt)
    {
      //auto* ppkt = std::dynamic_pointer_cast<VESC::packets::SetMCConf>(pkt);
      spdlog::info("[VESC][{}<={}] COMM_SET_MCCONF OK", this->id, src_id);
      return true;
    });*/
  }
}

std::chrono::milliseconds VESCpp::msSinceLastCANPkt() const
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(Time::now() - _last_can_t);
}

std::chrono::milliseconds VESCpp::msSinceLastVESCkt() const
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(Time::now() - _last_pkt_t);
}

bool VESCpp::send(Comm* comm, const VESC::BoardId tgt_id, VESC::Packet& pkt, uint8_t send_cmd)
{
  if(!comm)
    comm = _comm;

  if(auto* can = dynamic_cast<comm::CAN*>(comm))
    return sendCAN(can, tgt_id, pkt, send_cmd);
  return false;
}

bool VESCpp::pktProcess(Comm* comm, const VESC::BoardId src_id, std::shared_ptr<VESC::Packet>& pkt)
{
  if(VESCDevice::pktProcess(comm, src_id, pkt))
  {
    spdlog::trace("[VESCpp::pktProcess][{}<={}] Self handled Packet {}", id, src_id, pkt->id);
    return true;
  }

  if(auto it = _devs.find(src_id); it != _devs.end())
  {
    spdlog::trace("[VESCpp::pktProcess][{}<={}] Forward to device {}", id, src_id, pkt->id);
    return it->second->pktProcess(comm, src_id, pkt);
  }

  spdlog::debug("[VESCpp::pktProcess][{}<={}] Unhandled Packet {}", id, src_id, pkt->id);
  return false;
}

bool VESCpp::processRawPacket(Comm* comm, const VESC::BoardId src_id, const DataBuffer& buff, size_t start, size_t len)
{
  _last_pkt_t = Time::now();
  spdlog::trace("[{}<={}] Process raw packet", id, src_id, len);
  spdlog::trace("    => {:np}", spdlog::to_hex(buff));  
  if(len < 1)
  {
    spdlog::error("[{}<={}] Can't process empty packet", id, src_id);
    return false;
  }
  auto pkt_id = (VESC::PktId)buff[0];
  std::shared_ptr<VESC::Packet> pkt = std::move(VESC::packets::create(pkt_id, buff, start, len));
  if(!pkt)
  {
    spdlog::error("[{}<={}] Unknown packet with ID: {}", id, src_id, pkt_id);
    return false;
  }
  spdlog::trace("Created packet for ID: {}, {}", pkt_id, fmt::ptr(pkt));
  bool r = pktProcess(comm, src_id, pkt);
  if(!r)
    spdlog::error("[VESCpp::processRawPacket][{}<={}] Could not process Packet with ID: {}", id, src_id, pkt_id);
  return r;
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
      std::this_thread::sleep_for(_tx_delay_us); // Small delay to avoid overfilling the TX buffer
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

std::vector<std::pair<VESC::BoardId, VESC::HwTypeId>> VESCpp::scanCAN(bool add_devices, std::chrono::milliseconds timeout_ms)
{
  auto* can = dynamic_cast<comm::CAN*>(_comm);
  if(!can)
    return {};
  spdlog::trace("[{}] Scan CAN bus for VESC boards", id);
  std::vector<std::pair<VESC::BoardId, VESC::HwTypeId>> out;
  Time::time_point last_pong_time=Time::now();
  auto hdlr_id = can->addHandler((::VESC::CAN_PACKET_PONG<<8)|id,
    [this, &last_pong_time, &out](comm::CAN* can, const comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len)
    {
      last_pong_time = Time::now();
      spdlog::trace("[{}] Got CAN PONG: {:pn}", id, spdlog::to_hex(data, data+len));
      out.emplace_back((VESC::BoardId)(data[0]), (VESC::HwTypeId)(data[1]));
    });
  spdlog::trace("[{}] Sending PINGs...", id);
  for(uint8_t board_id=0;board_id<0xFF;board_id++)
  {
    uint8_t data = id&0xFF;
    spdlog::trace("[{}=>{}] Send PING", id, board_id);
    if(!can->write((::VESC::CAN_PACKET_PING<<8)| board_id, &data, 1))
    {
      spdlog::error("[{}=>{}] Error when sending PING", id, board_id);
    }
    std::this_thread::sleep_for(_tx_delay_us); // Small delay to avoid overfilling the TX buffer
  }
  spdlog::trace("[{}] Done sending PINGs", id);

  while (Time::now()-last_pong_time < timeout_ms)
    std::this_thread::sleep_for(1ms);
  spdlog::trace("[{}] Done scanning CAN bus for VESC boards", id);
  can->removeHandler(hdlr_id);
  if(add_devices)
  {
    for(const auto& [id,typ]: out)
    {
      if(auto v = this->add_peer(id,typ,100ms); v != nullptr)
      {
        const auto& fw = v->fw();           
        spdlog::debug("[{0}/0x{0:02X}] FW version: {1}.{2} - HW: {3:<15s} - UUID: 0x {4:spn}", id, fw->fw_version_major, fw->fw_version_minor,  fw->hw_name.c_str(), spdlog::to_hex(fw->uuid)); 
      }
    }
  }
  
  return out;
}

void VESCpp::processShortBufferCB(comm::CAN* can, const comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len)
{
  _last_can_t = Time::now();
  spdlog::trace("[{}] Got CAN_PACKET_PROCESS_SHORT_BUFFER: {:pn}", id, spdlog::to_hex(data, data+len));
  if(len < 3)
    return;

  const auto& tgt_id = can_id&0xFF;

  if(tgt_id != id)
    return;

  const auto& src_id = data[0];
                    // data[1] ???
  const auto& pkt_id = data[2];
  const auto& pkt_len = len-2;
  DataBuffer b; b.resize(pkt_len);
  for(size_t i=0;i<pkt_len;i++)
    b[i] = data[i+2];

  if(!processRawPacket(can, src_id, b, 0, pkt_len))
    spdlog::error("[VESCpp::processShortBufferCB][{}<={}] Could not process Short Packet {}", id, src_id, tgt_id, pkt_id);
}

void VESCpp::fillRXBufferCB(comm::CAN* can, const comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len)
{
  _last_can_t = Time::now();
  const bool isLong = ((can_id&0xFF00)>>8) == ::VESC::CAN_PACKET_FILL_RX_BUFFER_LONG;
  const auto& header_len = (isLong ? 2 : 1);
  spdlog::trace("[{}] Got {}      : {:pn}", id, isLong ? "CAN_PACKET_FILL_RX_BUFFER_LONG" : "CAN_PACKET_FILL_RX_BUFFER", spdlog::to_hex(data, data+len));
  uint16_t rx_idx = data[0];
  if(isLong)
    rx_idx = (rx_idx << 8) | data[1] ;
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
  _last_can_t = Time::now();
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
    spdlog::debug("[{}] Can't process packet. Unknown packet with length {}", id, rx_len);
    for(const auto& p: _rx_pkts)
    spdlog::debug("  available pkt, last_t: {}, state: {}, index: {}, data: {:np}", p.last_t.time_since_epoch().count(), (int)p.state, p.index, spdlog::to_hex(p.buffer));
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
            spdlog::error("[VESCpp::processRXBufferCB][{}<={}] Could not process Packet {}", id, src_id, pkt_idx);
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
  _last_can_t = Time::now();
  spdlog::trace("[{}] Got CAN_PACKET_PONG                : {:pn}", id, spdlog::to_hex(data, data+len));
}

void VESCpp::pingCB(comm::CAN* can, const comm::CAN::Id can_id, const uint8_t data[8], const uint8_t len)
{
  _last_can_t = Time::now();
  spdlog::trace("[{}] Got CAN_PACKET_PING                : {:pn}", id, spdlog::to_hex(data, data+len));
  const auto& src_id = len < 1 ? 0x00 : data[0];

  uint8_t tx_data[2] = { id, _fw->hw_type_vesc};
  can->write((::VESC::CAN_PACKET_PONG<<8)|src_id, tx_data, 2);
}

}