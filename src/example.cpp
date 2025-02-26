#include <lyra/lyra.hpp>

#include "vescpp/vescpp.hpp"
#include "vescpp/comm/can.hpp"

int main(int argc, char**argv) 
{
    spdlog::cfg::load_env_levels();

    bool show_help = false;
    std::string can_port = "can0";
    auto cli = lyra::help(show_help).description("VescPP example")
    | lyra::opt( can_port, "port")
        ["-P"]["--port"]
        ("CAN port to use")
    ;
    
    if (auto result = cli.parse( { argc, argv } ); !result)
    {
        spdlog::error("Error when parsing command line: {}. Abort", result.message());
        return 1;
    } 

    if(show_help)
    {
        std::cout << cli << std::endl; 
        return 0;
    }

    auto can_comm = vescpp::comm::CAN(can_port, 179);
    const auto& can_ids = can_comm.scan(std::chrono::milliseconds(100));

    auto vesc = vescpp::VESCpp(179, &can_comm);
    for(const auto& [id,typ]: can_ids)
    {
        vesc.add_peer(id,typ);
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    for(auto& [id, v]: vesc._devs)
    {
        if(!v->fw.is_valid)
            continue;
            
        spdlog::info("[0x{0}/0x{0:02X}] FW version: {1}.{2} - HW: {3:<15s} - UUID: 0x{4:spn}", id, v->fw.fw_version_major, v->fw.fw_version_minor,  v->fw.hw_name.c_str(), spdlog::to_hex(v->fw.uuid));
    }
    getchar();
    return EXIT_SUCCESS; 
}