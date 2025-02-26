#include <lyra/lyra.hpp>
#include "spdlog/cfg/env.h"

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
    auto can_ids = can_comm.scan(std::chrono::milliseconds(5000));

    auto vesc = vescpp::VESCpp(179, &can_comm);
    for(const auto& id: can_ids)
        vesc.add_peer(id);

    return 0;
}