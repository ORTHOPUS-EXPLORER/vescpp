cmake_minimum_required(VERSION 3.22)
project(vescpp)

option(VESCPP_BUILD_EXAMPLE "Build example application" ON)
option(VESCPP_BUILD_TESTS   "Build unittests"           ON)

include(FetchContent)
FetchContent_Declare(spdlog
  GIT_REPOSITORY  https://github.com/gabime/spdlog
  GIT_TAG         v1.10.0
)
FetchContent_Declare(nlohmann_json
  GIT_REPOSITORY  https://github.com/nlohmann/json
  GIT_TAG         v3.11.3
  GIT_PROGRESS TRUE  GIT_SHALLOW TRUE
  FIND_PACKAGE_ARGS
)

FetchContent_MakeAvailable(
  spdlog
  nlohmann_json
)

# VESCpp
add_library(${PROJECT_NAME} STATIC
  src/vescpp.cpp
  src/vescdevice.cpp
  src/comm/can.cpp
  src/vesc/utils.cpp
  src/packet/packet.cpp
  src/packet/vesc_packets.cpp
  src/packet/vesc_packets/000_fwversion.cpp
  src/packet/vesc_packets/013_014_016_017_093_095_conf.cpp
  src/packet/vesc_packets/020_021_terminal.cpp
)

target_include_directories(${PROJECT_NAME}
PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/include/
)

target_link_libraries(${PROJECT_NAME}
PUBLIC
  spdlog
  nlohmann_json::nlohmann_json
)

set_target_properties(${PROJECT_NAME} 
PROPERTIES
  CXX_STANDARD 17
)

if(VESCPP_BUILD_EXAMPLE)
  add_subdirectory(example)
  add_subdirectory(cli)
endif()

if(VESCPP_BUILD_TESTS)
  add_subdirectory(tests)
endif()