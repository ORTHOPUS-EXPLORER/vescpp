cmake_minimum_required(VERSION 3.22)
project(vescpp_cli)

find_package(spdlog REQUIRED)
find_package(lyra REQUIRED)
find_package(nlohmann_json REQUIRED)

# VESCpp CLI
add_executable(${PROJECT_NAME}
  cli.cpp
)

target_link_libraries(${PROJECT_NAME}
PRIVATE
  vescpp
  lyra
  spdlog::spdlog
  nlohmann_json::nlohmann_json
)

install(TARGETS 
  ${PROJECT_NAME}
)