cmake_minimum_required(VERSION 3.22)
project(vescpp_cli)

if(NOT TARGET spdlog)
  find_package(spdlog REQUIRED)
endif()
if(NOT TARGET lyra)
  find_package(lyra REQUIRED)
endif()
if(NOT TARGET nlohmann_json::nlohmann_json)
  find_package(nlohmann_json REQUIRED)
endif()

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