#include <fstream>

#include "vescpp/vescpp.hpp"

extern "C"
{
#include "heatshrink_encoder.h"
}

using namespace std::chrono_literals;

namespace vescpp
{

VESCTarget::VESCTarget(const VESC::BoardId id, VESCpp* host)
: VESCBase(id, host)
, _fw(nullptr)
{
  pktAddHandler(::VESC::COMM_FW_VERSION,[this](Comm* comm, const VESC::BoardId src_id, std::shared_ptr<VESC::Packet>& pkt)
  {
    //spdlog::debug("[VESC][{}/{}] Handling FwVersion", this->id, src_id);
    _fw = std::dynamic_pointer_cast<VESC::packets::FwVersion>(pkt);
    return true;
  });
  pktAddHandler(::VESC::COMM_PRINT,[this](Comm* comm, const VESC::BoardId src_id, std::shared_ptr<VESC::Packet>& pkt)
  {
    auto ppkt = std::dynamic_pointer_cast<VESC::packets::Print>(pkt);
    //spdlog::debug("[VESC][{}/{}] Handling Print", this->id, src_id);
    spdlog::info("[VESC][{}]<= {}", this->id, ppkt->str);
    return true;
  });

  pktAddHandler(::VESC::COMM_SET_MCCONF,[this](Comm* comm, const VESC::BoardId src_id, std::shared_ptr<VESC::Packet>& pkt)
  {
    auto ppkt = std::dynamic_pointer_cast<VESC::packets::SetMCConf>(pkt);
    if(pkt->isAck)
    {
      spdlog::info("[VESC][{}<={}] COMM_SET_MCCONF OK", this->id, src_id);
      return true;
    }
    return false;
  });
  pktAddHandler(::VESC::COMM_SET_APPCONF,[this](Comm* comm, const VESC::BoardId src_id, std::shared_ptr<VESC::Packet>& pkt)
  {
    //auto ppkt = std::dynamic_pointer_cast<VESC::packets::SetAppConf>(pkt);
    if(pkt->isAck)
    {
      spdlog::info("[VESC][{}<={}] COMM_SET_APPCONF OK", this->id, src_id);
      return true;
    }
    return false;
  });
  pktAddHandler(::VESC::COMM_SET_CUSTOM_CONFIG,[this](Comm* comm, const VESC::BoardId src_id, std::shared_ptr<VESC::Packet>& pkt)
  {
    //auto ppkt = std::dynamic_pointer_cast<VESC::packets::SetCustomConf>(pkt);
    if(pkt->isAck)
    {
      spdlog::info("[VESC][{}<={}] COMM_SET_CUSTOM_CONFIG OK", this->id, src_id);
      return true;
    }
    return false;
  });
}

bool VESCTarget::sendCmd(const std::string_view& cmd, std::chrono::milliseconds delay_after_send)
{
  spdlog::info("[VESC][{}]=> {}", id, cmd);
  vescpp::VESC::packets::TerminalCmd pkt(cmd);
  bool r = this->send(pkt);
  if(!r)
    spdlog::error("[VESC][{}] => {}, send error", id, cmd);
  std::this_thread::sleep_for(delay_after_send);
  return r;
}

bool readHex(const std::string& hex_filename, std::map<uint32_t, std::vector<uint8_t>>& hex_data);
bool readBin(const std::string& bin_filename, std::vector<uint8_t>& bin_data);


bool VESCTarget::eraseFirmware(bool isBootloader, bool fwdCan, bool isLzo, std::chrono::milliseconds delay_after_send, size_t fwSize)
{
  auto expect_id = ::VESC::COMM_ERASE_NEW_APP;
  if(isBootloader)
  {
    // FIXME: Support all hardware
    const auto id = fwdCan ? ::VESC::COMM_ERASE_BOOTLOADER_ALL_CAN : ::VESC::COMM_ERASE_BOOTLOADER;
    auto pkt = vescpp::VESC::RawPacket(id,{});
    expect_id = ::VESC::COMM_ERASE_BOOTLOADER;
    this->send(pkt);
  }
  else
  {
    const auto id = fwdCan ? ::VESC::COMM_ERASE_NEW_APP_ALL_CAN : ::VESC::COMM_ERASE_NEW_APP;
    auto pkt = vescpp::VESC::RawPacket(id,{});
    pkt.data.push_back((fwSize >>24)& 0xFF);
    pkt.data.push_back((fwSize >>16)& 0xFF);
    pkt.data.push_back((fwSize >> 8)& 0xFF);
    pkt.data.push_back((fwSize     )& 0xFF);
    this->send(pkt);
  }
  if(auto pkt = this->waitFor(expect_id, 20s))
  {
    if(pkt->data.size() != 1 || pkt->data[0] != 1)
    {
      spdlog::error("Erase failed. Abort");    
      return false;
    }
    spdlog::info("Erase OK");
    return true;
  }
  spdlog::error("Erase Timeout");
  return false;
}


bool VESCTarget::flashFirmware(const std::string& fw_file, bool isBootloader, bool fwdCan, bool isLzo)
{
  size_t fwSize=0;
  std::vector<uint8_t> fwData;
  uint32_t fwOffset = 0;
  uint32_t addr = 0;
  if(false)
  {
    std::map<uint32_t, std::vector<uint8_t>> hex_data;
    if(!readHex(fw_file,hex_data))
    {
      spdlog::error("Can't read Hex file {:s}", fw_file);
      return false;
    }
    for(const auto& [addr, data]: hex_data)
    {
      fwSize += data.size();
      fwData.insert(fwData.end(), data.begin(), data.end());

      spdlog::debug("Addr {:08X}, Data {:8d}", addr, data.size());//spdlog::to_hex(data));
    }
  }
  else
  {
    if(!readBin(fw_file,fwData))
    {
      spdlog::error("Can't read bin file {:s}", fw_file);
      return false;
    }
    fwSize = fwData.size();
  }

  // FIXME: Support all hardware
  if(fw()->hw_type_vesc != ::VESC::HW_TYPE_VESC)
  {
    spdlog::error("Only VESC HW firmware upgrade is currently supported. Abort");
    return false;
  }

  // FIXME: Support FwdCan
  if(fwdCan)
  {
    spdlog::error("CAN forward flash is not currently supported. Abort");
    return false;
  }

  // FIXME: Support LZO
  bool supportLzo = false;
  if(isLzo)
  {
    spdlog::error("Compressed flash is not currently supported. Abort");
    return false;
  }

  // FIXME: Support Bootloader flash
  if(isBootloader)
  {
    switch(fw()->hw_type_vesc)
    {
      case ::VESC::HW_TYPE_VESC:
        addr += 0x60000; // 1024*128*3 (BL is in FLASH_SECTOR_7 ?)
        break;
      default:
        // FIXME: Support all hardware
        break;
    }
    spdlog::error("Bootloader flash is not currently supported. Abort");
    return false;
  }

  spdlog::debug("Erase firmware");
  if(!this->eraseFirmware(isBootloader, fwdCan, isLzo, 1s, fwSize))
  {
    spdlog::error("Firmware erase failed");
    return false;
  }
  
  spdlog::debug("Send new firmware, file: {:s}, size {:d}, offset 0x{:08x}",fw_file, fwSize, fwOffset);
  size_t startAddr = addr;
  bool useHS = false;
  if (fwSize > 393208 && fwSize < 700000) 
  {
    useHS = true;
    spdlog::debug("Firmware is quite big, compress it first !");
    heatshrink_encoder hse;
    heatshrink_encoder_reset(&hse);
    size_t hsSize = fwSize + fwSize/2 + 4; // Wut ?
    std::vector<uint8_t> fwDataHS; fwDataHS.resize(hsSize);
    size_t sunk = 0,
           count = 0,
           fwSizeHS = 0;
    while(sunk < fwSize)
    {
      heatshrink_encoder_sink(&hse, &fwData[sunk], fwSize-sunk, &count);
      sunk += count;
      if (sunk == fwSize)
        heatshrink_encoder_finish(&hse);

      HSE_poll_res pres;
      do 
      {
          pres = heatshrink_encoder_poll(&hse, &fwDataHS[fwSizeHS], hsSize-fwSizeHS, &count);
          fwSizeHS += count;
      } while (pres == HSER_POLL_MORE);

      if (sunk == fwSize)
          heatshrink_encoder_finish(&hse);
    }
    fwDataHS.resize(fwSizeHS);

    supportLzo = false;
    const auto fwSizeOrig = fwSize;
    const auto fwDataOrig = std::move(fwData);
    fwSize = fwSizeHS;
    fwData = std::move(fwDataHS);

    spdlog::debug("Firmware heatshrunk from {:d} to {:d} bytes ({:.2f}%)",fwSizeOrig, fwSize, (100.0*fwSize/fwSizeOrig));
    if(fwSize > 393208)
    {
       spdlog::error("Heatshrunk firmware is still too big! Abort");
       return false;
    }
  }
  
  if(!isBootloader)
  {
    // uint16_t crc16(const vescpp::DataBuffer& buf, size_t start=0, size_t len=0);
    auto fwCRC = ::VESC::crc16(fwData);
    std::vector<uint8_t> fwHeader;
    uint32_t shift = fwSize;
    if(useHS)
      shift |= 0xCC<<24;

    fwHeader.push_back((shift >> 24)&0xFF);
    fwHeader.push_back((shift >> 16)&0xFF);
    fwHeader.push_back((shift >> 8)&0xFF);
    fwHeader.push_back(shift&0xFF);
    fwHeader.push_back(fwCRC >> 8);
    fwHeader.push_back(fwCRC&0xFF);
    spdlog::debug("Size: {:d}/{:d}, 0x{:06x} CRC: 0x{:04x}, fwHeader: {:np}", fwSize, fwData.size(), fwSize, fwCRC, spdlog::to_hex(fwHeader));
    fwData.insert(fwData.begin(),fwHeader.begin(),fwHeader.end());
    fwSize = fwData.size();
  }

  size_t index=0;
  const auto chunkSz = 384;
  while(index < fwSize)
  {
    const auto sz = fwSize-index > chunkSz ? chunkSz : fwSize-index;
    std::vector<uint8_t> data; data.insert(data.begin(), fwData.begin()+index, fwData.begin()+index+sz);
    //spdlog::trace("[{}/{}][0x{:08X}] Send {:d} bytes...", index, fwSize, addr, sz);//, spdlog::to_hex(data));
    bool hasData = false;
    for(const auto v: data)
    {
      if(v != 0xFF)
      {
        hasData = true;
        break;
      }
    }

    int res = 1;
    if(hasData)
    {
      const auto maxSz  = chunkSz + chunkSz/16 + 64 + 3; // Wuuut ?
      if(isLzo && supportLzo)
      {
        // FIXME: Support LZO
        spdlog::error("LSO not supported. Abort");
        return false;
      }
      else
      {
        // FIXME: Support all hardware
        const auto id = fwdCan ? ::VESC::COMM_WRITE_NEW_APP_DATA_ALL_CAN : ::VESC::COMM_WRITE_NEW_APP_DATA;
        auto pkt = vescpp::VESC::RawPacket(id,{});
        auto& pdata = pkt.data;
        pdata.push_back((addr >> 24)&0xFF);
        pdata.push_back((addr >> 16)&0xFF);
        pdata.push_back((addr >> 8)&0xFF);
        pdata.push_back( addr&0xFF);
        pdata.insert(pdata.end(),data.begin(), data.end());
        this->send(pkt);
        bool offset = 0x0;
        if(auto pkt = this->waitFor(::VESC::COMM_WRITE_NEW_APP_DATA, 1s))
        {
          if(pkt->data.size() < 1 || pkt->data[0] != 1)
          {
            spdlog::error("[{}/{}] Chunk Write failed", index, fwSize, addr);
            return false;
          }
          spdlog::trace("[{}/{}] Chunk Write OK at addr 0x{:08x}", index, fwSize, addr);
        }
        else
        {
          spdlog::error("[{}/{}] Chunk Write timeout", index, fwSize, addr);
          return false;
        }
      }
    }
    addr += sz;
    index += sz;
    //break;
  }

  if(!isBootloader)
  {
    spdlog::info("Jump to Bootloader");
    // FIXME: Support all hardware
    const auto id = fwdCan ? ::VESC::COMM_JUMP_TO_BOOTLOADER_ALL_CAN : ::VESC::COMM_JUMP_TO_BOOTLOADER;
    auto pkt = vescpp::VESC::RawPacket(id,{});
    this->send(pkt);
  }


  return true;
}


bool readBin(const std::string& bin_filename, std::vector<uint8_t>& bin_data)
{
  std::ifstream bin_file(bin_filename, std::ios::binary);
  bin_data = std::vector<uint8_t>(std::istreambuf_iterator<char>(bin_file), {});
  return true;
}


// Intentional breaks
// Addr                   size    , name
const std::unordered_map<uint32_t, std::pair<uint32_t, std::string>> mem_breaks =
{
  {0x08000000, {  16384, "FLASH_SECTOR_0 " } },
  {0x08004000, {  16384, "FLASH_SECTOR_1 " } },
  {0x08008000, {  16384, "FLASH_SECTOR_2 " } },
  {0x0800C000, {  16384, "FLASH_SECTOR_3 " } },
  {0x08010000, {  65536, "FLASH_SECTOR_4 " } },
  {0x08020000, { 131072, "FLASH_SECTOR_5 " } },
  {0x08040000, { 131072, "FLASH_SECTOR_6 " } },
  {0x08060000, { 131072, "FLASH_SECTOR_7 " } },
  {0x08080000, { 131072, "FLASH_SECTOR_8 " } },
  {0x080A0000, { 131072, "FLASH_SECTOR_9 " } },
  {0x080C0000, { 131072, "FLASH_SECTOR_10" } },
  {0x080E0000, { 131072, "FLASH_SECTOR_11" } },
  {0x1FFF0000, {  30720, "FLASH_SYS_MEM  " } },
  {0x1FFF7800, {    528, "FLASH_OTP      " } },
  {0x1FFFC000, {     16, "FLASH_OPTION   " } },
};

bool readHex(const std::string& hex_filename, std::map<uint32_t, std::vector<uint8_t>>& hex_data)
{
  std::ifstream hex_file(hex_filename);
  std::string line; size_t l=0;
  uint32_t base_addr = 0x0, next_addr = 0x0;
  std::vector<uint8_t>* current_bin_data=nullptr;
  while (std::getline (hex_file, line)) {
    //spdlog::trace("Hex line: {:s}", line);
    size_t c_i=1;
    const auto& l_sz = line.length()-1;

    if(line[0] != ':')
    {
      spdlog::error("readHex: Line {:d} doesn't start with ':'. Abort",l);
      spdlog::debug("  => {:s}", line);
      return false;
    }
    if((l_sz - c_i) < 10)
    {
      spdlog::error("readHex: Line {:d} doesn't have enough character count, Line length: {:d}. Abort",l,l_sz);
      spdlog::debug("  => {:s}", line);
      return false;
    }
    if((l_sz - c_i)%2 != 0)
    {
      spdlog::error("readHex: Line {:d} doesn't have an even character count, Line length: {:d}. Abort",l,l_sz);
      spdlog::debug("  => {:s}", line);
      return false;
    }
    uint8_t size = 0x00,
            type = 0x00,
            crc = 0x00;
    uint32_t addr = base_addr;
    std::vector<uint8_t> data; data.reserve(16);
    // Read line
    while(c_i < l_sz)
    {
      std::string sc = line.substr(c_i, 2);
      uint8_t c = std::stoul(sc, nullptr, 16);
      if(c_i == 1)
        size = c;
      else if(c_i == 3)
        addr |= (uint16_t)(c) << 8;
      else if(c_i == 5)
        addr |= c;
      else if(c_i == 7)
        type = c;
      else if(c_i == l_sz - 2)
        crc = c;
      else
        data.emplace_back(c);
      //spdlog::trace("{:2d}/{:2d}:{:02X} Address {:08X}, Type {:02X}, CRC {:02X}, Data ({:2d}) {:np}",c_i,l_sz,c, addr, typ, crc, data.size(), spdlog::to_hex(data));
      c_i += 2;
    }
    //spdlog::trace("Address {:08X}, Sz {:2d}, Type {:02X}, CRC {:02X}, Data ({:2d}) {:np}", addr, size, type, crc, data.size(), spdlog::to_hex(data));
    // Process line
    if(size != data.size())
    {
      spdlog::error("readHex: Line {:d} length {:d} does not match size {:d}. Abort",l,l_sz, size);
      spdlog::debug("  => {:s}", line);
      return false;
    }
    // TODO Check CRC
    switch(type)
    {
      case 0x04:  // ADDRESS Section
        if(size != 2)
        {
          spdlog::error("readHex: Line {:d}: Type 0x04 size is not 2 bytes. Abort",l);
          return false;
        }
        base_addr = (uint32_t)(((uint16_t)((uint8_t)data[0]) << 8 ) | (uint8_t)data[1]) << 16;
        next_addr = 0;
        break;
      case 0x05:
        spdlog::warn("readHex: Line {:d}: Type 0x05 not handled. ignore",l);
        break;
      case 0x00:  // DATA Section
        if(!size)
        {
          spdlog::error("readHex: Line {:d}: Type 0x00 size is too short. Abort",l);
          return false;
        }


        for(const auto& [m_addr, it]: mem_breaks)
        {
          if(addr == m_addr)// || addr == m_addr + it.first)
          {
            spdlog::trace("readHex: Line {:6d}. Add section break for {:s}, at {:08X}",l,it.second, addr);
            next_addr = 0;
            break;
          }
          //if(current_bin_data && current_bin_data->size() == m_addr + it.first)
          //{
          //  spdlog::trace("readHex: Line {:6d}. Current section overflows. Add section break for {:s}, at {:08X}",l,it.second, addr);
          //  next_addr = 0;
          //  break;
          //}
        }

        if(!next_addr || addr != next_addr)
        {
          //spdlog::trace("readHex: Line {:5d} New slice: Base: {:08X} Addr {:08X} ({:04X}) next: {:08X}, len: {:10d} ({:04X})",l,base_addr,addr,addr-base_addr,next_addr,addr-next_addr, addr-next_addr);

          auto r = hex_data.emplace(addr,std::vector<uint8_t>{});
          current_bin_data = &(r.first->second);
        }

        current_bin_data->reserve(current_bin_data->size()+data.size());
        for(auto& c: data)
          current_bin_data->emplace_back(c);
        next_addr = addr+size;
        break;
      case 0x01:  // EOF Section
        break;
      default:
        spdlog::error("readHex: Unhandled type: 0x{:02X}", type);
        break;
    };
    l++;
  }
  return true;
}

}

/*
 * Main memory
 * 0x08000000 - 0x08003FFF, Sector  0,  16k
 * 0x08004000 - 0x08007FFF, Sector  1,  16k
 * 0x08008000 - 0x0800BFFF, Sector  2,  16k
 * 0x0800C000 - 0x0800FFFF, Sector  3,  16k
 * 0x08010000 - 0x0801FFFF, Sector  4,  64k
 * 0x08020000 - 0x0803FFFF, Sector  5, 128k
 * 0x08040000 - 0x0805FFFF, Sector  6, 128k
 * 0x08060000 - 0x0807FFFF, Sector  7, 128k
 * 0x08080000 - 0x0809FFFF, Sector  8, 128k
 * 0x080A0000 - 0x080BFFFF, Sector  9, 128k
 * 0x080C0000 - 0x080DFFFF, Sector 10, 128k
 * 0x080E0000 - 0x080FFFFF, Sector 11, 128k
 * System memory
 * 0x1FFF0000 - 0x1FFF77FF, Sector  6,  30k
 * OTP Area
 * 0x1FFF7800 - 0x1FFF7A0F, Sector  7, 528b
 * Option bytes
 * 0x1FFFC000 - 0x1FFFC00F, Sector  7,  16b
 */
/*
  #define FLASH_ADDR_SECTOR_0  0x08000000
  #define FLASH_SIZE_SECTOR_0       16384
  #define FLASH_ADDR_SECTOR_1  0x08004000
  #define FLASH_SIZE_SECTOR_1       16384
  #define FLASH_ADDR_SECTOR_2  0x08008000
  #define FLASH_SIZE_SECTOR_2       16384
  #define FLASH_ADDR_SECTOR_3  0x0800C000
  #define FLASH_SIZE_SECTOR_3       16384
  #define FLASH_ADDR_SECTOR_4  0x08010000
  #define FLASH_SIZE_SECTOR_4       65536
  #define FLASH_ADDR_SECTOR_5  0x08020000
  #define FLASH_SIZE_SECTOR_5      131072
  #define FLASH_ADDR_SECTOR_6  0x08040000
  #define FLASH_SIZE_SECTOR_6      131072
  #define FLASH_ADDR_SECTOR_7  0x08060000
  #define FLASH_SIZE_SECTOR_7      131072
  #define FLASH_ADDR_SECTOR_8  0x08080000
  #define FLASH_SIZE_SECTOR_8      131072
  #define FLASH_ADDR_SECTOR_9  0x080A0000
  #define FLASH_SIZE_SECTOR_9      131072
  #define FLASH_ADDR_SECTOR_10 0x080C0000
  #define FLASH_SIZE_SECTOR_10     131072
  #define FLASH_ADDR_SECTOR_11 0x080E0000
  #define FLASH_SIZE_SECTOR_11     131072
  #define FLASH_ADDR_SYS_MEM   0x1FFF0000
  #define FLASH_SIZE_SYS_MEM        30720
  #define FLASH_ADDR_OTP       0x1FFF7800
  #define FLASH_SIZE_OTP              528
  #define FLASH_ADDR_OPTION    0x1FFFC000
  #define FLASH_SIZE_OPTION            16
*/
