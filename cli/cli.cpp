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
bool writeToFile(const std::string &name, vescpp::VESCDevice &vesc);

template <class GetPktT, class SetPktT>
bool loadFromFile(const std::string &name, vescpp::VESCDevice &vesc, bool force = false);

int main(int argc, char **argv)
{
  spdlog::cfg::load_env_levels();

  bool show_help = false,
       force_load = false,
       scan_bus = false;
  unsigned int host_id = 51,
               device_id = 11,
               exit_delay_ms = 100;
  std::string can_port = "can0",
              cmd = "",
              conf_type = "",
              conf_dir ="",
              conf_file = "";
  std::vector<std::string> cmd_args;

  auto cli = lyra::help(show_help).description("VESCpp CLI") 
  | lyra::opt(can_port, "port")["-P"]("CAN port (eg: can0)") 
  | lyra::opt(host_id, "id")["-I"]("Host ID (1-254)") 
  | lyra::opt(device_id, "id")["-i"]("Device ID (1-254)") 
  | lyra::opt(exit_delay_ms, "delay")["-x"]("Delay, in ms, after last CAN message before exiting CLI") 
  | lyra::opt(force_load)["-f"]("Force loading config") 
  | lyra::opt(conf_dir, "dir")["-D"]("Directory to load/files from") 
  | lyra::group().sequential() 
    | lyra::arg(cmd, "cmd").help("Command: save_conf, load_conf, proxy, scan").required()
    | lyra::arg(conf_type, "type").help("Type (when using save_conf/load_conf): app, motor, custom") 
    | lyra::arg(conf_file, "file").help("Input/Ouput file (when using save_conf/load_conf)") 
  | lyra::group() 
    | lyra::arg(cmd_args, "args").help("Command arguments");

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
  spdlog::info("[{}] Start VESCpp CLI with Host ID {}, device ID {}", can_port, host_id, device_id);

  auto can_comm = vescpp::comm::CAN(can_port);
  auto vescpp = vescpp::VESCpp(host_id, &can_comm, false);
  if(cmd == "scan")
  {
    const auto& can_ids = vescpp.scanCAN(10ms);
    for(const auto& [id,typ]: can_ids)
    {
      if(auto v = vescpp.add_peer(id,typ,100ms); v != nullptr)
      {
        const auto& fw = v->fw();           
        spdlog::info("[{0}/0x{0:02X}] FW version: {1}.{2} - HW: {3:<15s} - UUID: 0x{4:spn}", id, fw->fw_version_major, fw->fw_version_minor,  fw->hw_name.c_str(), spdlog::to_hex(fw->uuid)); 
      }
    }
    return 0;
  }

  auto vesc = vescpp.add_peer(device_id, ::VESC::HW_TYPE_VESC, 100ms);
  if (!vesc)
  {
    spdlog::error("Can't add peer. Abort");
    return EXIT_FAILURE;
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
    bool force_load = false;

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
  spdlog::debug("Ok, bye");
  return EXIT_SUCCESS;
}

template <class GetPktT>
bool writeToFile(const std::string &name, vescpp::VESCDevice &vesc)
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
bool loadFromFile(const std::string &name, vescpp::VESCDevice &vesc, bool force)
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
    if (!force && gpkt->signature != spkt.signature)
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