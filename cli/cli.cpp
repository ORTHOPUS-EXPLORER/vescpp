#include <lyra/lyra.hpp>

#include "vescpp/vescpp.hpp"
#include "vescpp/comm/can.hpp"

#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

using namespace std::chrono_literals;
using namespace vescpp;
using namespace vescpp::VESC;

template <class GetPktT>
bool writeToFile(const std::string &name, vescpp::VESCTarget &vesc);

template <class GetPktT, class SetPktT>
bool loadFromFile(const std::string &name, vescpp::VESCTarget &vesc, bool force = false);

int main(int argc, char **argv)
{
  spdlog::cfg::load_env_levels();

  bool show_help = false,
       force_load = false,
       use_single_dev = false;
  unsigned int host_id = 51,
               device_id = 0,
               exit_delay_ms = 100;
  std::string can_port = "can0",
              cmd = "",
              conf_type = "",
              conf_dir ="",
              conf_file = "",
              device_uuid = "";
  std::vector<std::string> cmd_args;

  auto cli = lyra::help(show_help).description("VESCpp CLI")
  | lyra::opt(can_port,       "port" )["-P"]("CAN port (eg: can0)")
  | lyra::opt(host_id,        "id"   )["-I"]("Host ID (1-254)")
  | lyra::opt(device_id,      "id"   )["-i"]("Device ID (1-254)")
  | lyra::opt(device_uuid,    "uuid" )["-u"]("Device UUID (hex, 12bytes)")
  | lyra::opt(exit_delay_ms,  "delay")["-x"]("Delay, in ms, after last CAN message before exiting CLI")
  | lyra::opt(force_load             )["-f"]("Force loading config")
  | lyra::opt(use_single_dev         )["-S"]("Use autodetected Device when there's only one on the CAN bus")
  | lyra::opt(conf_dir,       "dir"  )["-D"]("Directory to load/files from")
  | lyra::group().sequential()
    | lyra::arg(cmd,          "cmd").help("Command: device, save_conf, load_conf, proxy, scan").required()
    | lyra::arg(conf_type,    "type").help("Type (when using save_conf/load_conf): app, motor, custom")
    | lyra::arg(conf_file,    "file").help("Input/Ouput file (when using save_conf/load_conf/flash_fw)")
  | lyra::group()
    | lyra::arg(cmd_args,     "args").help("Command arguments");

  device_id &= 0xFF;

  if (auto result = cli.parse({argc, argv}); !result)
  {
    spdlog::error("Error when parsing command line: {}. Abort", result.message());
    return EXIT_FAILURE;
  }

  if (show_help || !cmd.length())
  {
    std::cout << cli << std::endl;
    return EXIT_SUCCESS;
  }

  std::shared_ptr<VESCDevice> vescpp(nullptr);
  auto can_comm = vescpp::comm::CAN(can_port);
  if(cmd == "device")
  {
    spdlog::info("[{}] Start VESCpp CLI, device mode, with Board ID {}", can_port, device_id);
    exit_delay_ms = 0;
    auto vescpp = vescpp::VESCDevice(device_id, &can_comm);
    spdlog::info("Press any key then Enter to exit");
    getchar();
  }
  else
  {
    spdlog::info("[{}] Start VESCpp CLI with Host ID {}", can_port, host_id);
    auto vescpp = vescpp::VESCHost(host_id, &can_comm);
    if(cmd == "scan")
    {
      vescpp.scanCAN(true, 100ms);
      const auto& peers = vescpp.peers();
      for(const auto& [_, v]: peers)
      {
        const auto& fw = v->fw();
        spdlog::info("[{0}/0x{0:02X}] FW version: {1}.{2} - HW: {3:<15s} - UUID: 0x {4:spn}", v->id, fw->fw_version_major, fw->fw_version_minor,  fw->hw_name.c_str(), spdlog::to_hex(fw->uuid));
      }
      spdlog::debug("OK bye, scan");
      return EXIT_SUCCESS;
    }

    std::shared_ptr<VESCTarget> vesc(nullptr);
    // use provided ID
    spdlog::info("[{}][{}] Look for Target ID {}", can_port, host_id, device_id);
    if(device_id > 0)
      vesc = vescpp.add_peer(device_id, ::VESC::HW_TYPE_VESC, 100ms);
    if(!vesc)
    {
      vescpp.scanCAN(true, 10ms);
      const auto& peers = vescpp.peers();
      // Find with UUID
      if(device_uuid.length())
      {
        spdlog::info("[{}][{}] ID not found, look for Target UUID {}", can_port, host_id, device_uuid);
        for(const auto& [id, v]: peers)
        {
          const auto& fw = v->fw();
          const auto duuid = fmt::format("0x{:spn}", spdlog::to_hex(fw->uuid));
          if(duuid == device_uuid)
          {
            vesc = v;
            break;
          }
        }
      }
      // Fallback
      if(!vesc && peers.size() == 1)
      {
        vesc = peers.begin()->second;
        spdlog::info("[{}][{}] Fallback to the only connected device: {}", can_port, host_id, vesc->id);
      }
    }

    if(!vesc)
    {
      spdlog::error("Can't find device with ID '{}' or UUID '{}'. Abort", device_id, device_uuid);
      return EXIT_FAILURE;
    }

    {
      const auto& fw = vesc->fw();
      spdlog::info("[{0}/0x{0:02X}] FW version: {1}.{2} - HW: {3:<15s} - UUID: 0x{4:spn}", vesc->id, fw->fw_version_major, fw->fw_version_minor,  fw->hw_name.c_str(), spdlog::to_hex(fw->uuid));
    }


    if (cmd == "proxy")
    {
      std::string s;
      // FIXME: quite fugly, figure out how Lyra works
      s += conf_type + " ";
      s += conf_file + " ";
      // Add remaining args
      for (auto &a : cmd_args)
        s += a + " ";
      // Trim spaces
      std::string_view sw(s);
      sw.remove_suffix(std::distance(sw.crbegin(), std::find_if(sw.crbegin(), sw.crend(),
                                                                [](int c)
                                                                { return !std::isspace(c); })));
      // Send command
      vesc->sendCmd(sw);
    }
    else if (cmd == "load_conf")
    {
      std::unique_ptr<vescpp::VESC::Packet> pkt;
      //bool force_load = false;

      if (!conf_file.length())
      {
        spdlog::error("No input file specified, abort");
        return -1;
      }
      if(conf_dir.length())
        conf_file = conf_dir+"/"+conf_file;

      spdlog::info("Loading Config from {}", conf_file);
      if (conf_type == "app" && loadFromFile<packets::GetAppConf, packets::SetAppConf>(conf_file, *vesc, force_load))
        spdlog::info("Loaded App config to {}", conf_file);
      else if (conf_type == "motor" && loadFromFile<packets::GetMCConf, packets::SetMCConf>(conf_file, *vesc, force_load))
        spdlog::info("Loaded Motor config to {}", conf_file);
      else if (conf_type == "custom" && loadFromFile<packets::GetCustomConf, packets::SetCustomConf>(conf_file, *vesc, force_load))
        spdlog::info("Loaded Custom config to {}", conf_file);
    }
    else if (cmd == "save_conf")
    {
      if (!conf_file.length())
      {
        auto t = std::time(nullptr);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S");
        const auto now_s = ss.str();
        conf_file = conf_type + "_conf_"+std::to_string(vesc->id)+"_"+ now_s + ".json";
      }
      if(conf_dir.length())
      {
        std::filesystem::create_directories(conf_dir);
        conf_file = conf_dir+"/"+conf_file;
      }

      if (conf_type == "app" && writeToFile<packets::GetAppConf>(conf_file, *vesc))
        spdlog::info("Saved App config to {}", conf_file);
      else if (conf_type == "motor" && writeToFile<packets::GetMCConf>(conf_file, *vesc))
        spdlog::info("Saved Motor config to {}", conf_file);
      else if (conf_type == "custom" && writeToFile<packets::GetCustomConf>(conf_file, *vesc))
        spdlog::info("Saved Custom config to {}", conf_file);
    }
    else if (cmd == "flash_fw")
    {
      if (!conf_file.length())
      {
        spdlog::error("No input file specified, abort");
        return -1;
      }
      if(conf_dir.length())
        conf_file = conf_dir+"/"+conf_file;
      if(!vesc->flashFirmware(conf_file, conf_type == "bootloader", false, false))
        spdlog::error("Firmware Flash failed with: {}", conf_file);
      else
        spdlog::info("Firmware Flash OK", conf_file);

    }
    else
      spdlog::error("Unkown command: {}", cmd);

    if (exit_delay_ms)
    {
      while (vescpp.msSinceLastCANPkt() < std::chrono::milliseconds(exit_delay_ms))
        std::this_thread::sleep_for(1ms);
    }
    else
    {
      spdlog::info("Press any key then Enter to exit");
      getchar();
    }
  }

  spdlog::debug("Ok, bye");
  return EXIT_SUCCESS;
}

template <class GetPktT>
bool writeToFile(const std::string &name, vescpp::VESCTarget &vesc)
{
  if (auto cnf = vesc.request<GetPktT>(200ms))
  {
    spdlog::info("Saving Config to {}", name);
    json j;
    j["board_id"] = vesc.id;
    j["info"] = vesc.fw()->toJson();
    j["data"] = cnf->toJson();
    std::ofstream ofile(name);
    ofile << j.dump(2);
    return true;
  }
  return false;
}

template <class GetPktT, class SetPktT>
bool loadFromFile(const std::string &name, vescpp::VESCTarget &vesc, bool force)
{
  SetPktT spkt;
  std::ifstream ifile(name);
  json j = json::parse(ifile);
  if (!force && vesc.id != j["board_id"])
    spdlog::warn("Board ID mismatch");
  packets::FwVersion fwpkt;
  fwpkt.fromJson(j["info"]);
  if (vesc.fw()->uuid != fwpkt.uuid)
  {
    if(!force)
    {
      spdlog::error("UUID mismatch, abort");
      return false;
    }
    spdlog::warn("UUID mismatch, continue anyway");
  }
  if (!spkt.fromJson(j["data"]))
  {
    spdlog::error("Can't create Packet from File");
    return false;
  }

  // Get current conf to match signature
  if (auto gpkt = vesc.request<GetPktT>(200ms))
  {
    if (gpkt->signature != spkt.signature)
    {
      if(!force)
      {
        spdlog::error("Signatures mismatch, abort");
        return false;
      }
      spdlog::warn("Signatures mismatch, continue anyway");
    }
    vesc.send(spkt);
    if (auto cpkt = vesc.waitFor<SetPktT>(); cpkt && cpkt->isAck)
    {
      // spdlog::debug("Config updated from {}", name);
      return true;
    }
    spdlog::debug("Sent Config from {}, but have not heard back from device", name);
    return true;
  }
  return false;
}
