#include <lyra/lyra.hpp>

#include "vescpp/vescpp.hpp"
#include "vescpp/comm/can.hpp"

using namespace std::chrono_literals;

int main(int argc, char**argv)
{
    spdlog::cfg::load_env_levels();

    bool show_help = false;
    std::string can_port = "can0";
    int board_id = 51;
    bool device_mode = false;
    auto cli = lyra::help(show_help).description("VESCpp example")
    | lyra::opt( can_port, "port")
        ["-P"]["--port"]
        ("CAN port")
    | lyra::opt( board_id, "board_id")
        ["-i"]["--board-id"]
        ("Device ID")
    | lyra::opt( device_mode)
        ["-m"]["--device-mode"]
        ("Act as device (VESC-like)")
    ;
    board_id &= 0xFF;

    if (auto result = cli.parse( { argc, argv } ); !result)
    {
        spdlog::error("Error when parsing command line: {}. Abort", result.message());
        return EXIT_FAILURE;
    }

    if(show_help)
    {
        std::cout << cli << std::endl;
        return EXIT_SUCCESS;
    }
    spdlog::info("[{}] Start VESCpp with ID {}, device_mode: {}", can_port, board_id, device_mode);

    auto can_comm = vescpp::comm::CAN(can_port);
    if(!device_mode)
    {
        auto vescpp = vescpp::VESCHost(board_id, &can_comm);
        vescpp.scanCAN(true, 10ms);
        const auto& peers = vescpp.peers();
        for(const auto& [_, v]: peers)
        {
          const auto& fw = v->fw();
          spdlog::info("[{0}/0x{0:02X}] FW version: {1}.{2} - HW: {3:<15s} - UUID: 0x {4:spn}", v->id, fw->fw_version_major, fw->fw_version_minor,  fw->hw_name.c_str(), spdlog::to_hex(fw->uuid));
        }
        spdlog::info("Press enter to exit");
        getchar();
    }
    else
    {
        auto vescpp = vescpp::VESCDevice(board_id, &can_comm);
        spdlog::info("Press enter to exit");
        getchar();
    }
    
    return EXIT_SUCCESS;
}
