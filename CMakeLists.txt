cmake_minimum_required(VERSION 3.22)
project(vescpp)

option(VESCPP_BUILD_EXAMPLE "Build example application" ON)
option(VESCPP_BUILD_TESTS   "Build unittests"           ON)

include(FetchContent)
set(EXT_PKGS)
if(NOT TARGET spdlog)
  find_package(spdlog QUIET)
  if(NOT spdlog_FOUND)
    FetchContent_Declare(spdlog
      GIT_REPOSITORY  https://github.com/gabime/spdlog
      GIT_TAG         v1.10.0
      GIT_PROGRESS TRUE  GIT_SHALLOW TRUE
      FIND_PACKAGE_ARGS
    )
    list(APPEND EXT_PKGS spdlog)
    message(STATUS "[+] Use FetchContent to get spdlog v1.10.0")
  else()
    message(STATUS "[+] Use System-provided spdlog ${spdlog_VERSION}")
  endif()
endif()
if(NOT TARGET nlohmann_json)
  find_package(nlohmann_json QUIET)
  if(NOT nlohmann_json_FOUND)
    FetchContent_Declare(nlohmann_json
      GIT_REPOSITORY  https://github.com/nlohmann/json
      GIT_TAG         v3.11.3
      GIT_PROGRESS TRUE  GIT_SHALLOW TRUE
      FIND_PACKAGE_ARGS
    )
    list(APPEND EXT_PKGS nlohmann_json)
    message(STATUS "[+] Use FetchContent to get nlohmann_json v3.11.3")
  else()
    message(STATUS "[+] Use System-provided nlohmann_json ${nlohmann_json_VERSION}")
  endif()
endif()
if(EXT_PKGS)
  FetchContent_MakeAvailable(
    ${EXT_PKGS}
  )
endif()

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
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
PUBLIC
  spdlog::spdlog
  nlohmann_json::nlohmann_json
)

set_target_properties(${PROJECT_NAME} 
PROPERTIES
  CXX_STANDARD 17
)

install(
  DIRECTORY   include/
  DESTINATION include
)

if(VESCPP_BUILD_EXAMPLE)
  # Lyra is used in all examples so fetch it here
  if(NOT TARGET lyra)
    find_package(lyra QUIET)
    if(NOT lyra_FOUND)
      FetchContent_Declare(lyra
        GIT_REPOSITORY  https://github.com/bfgroup/Lyra
        GIT_TAG         1.6.1
        GIT_PROGRESS TRUE  GIT_SHALLOW TRUE
        FIND_PACKAGE_ARGS
      )
      FetchContent_MakeAvailable(lyra)
      message(STATUS "[+] Use FetchContent to get lyra 1.6.1")
    else()
      message(STATUS "[+] Use System-provided lyra ${lyra_VERSION}")
    endif()
  endif()
  add_subdirectory(example)
  add_subdirectory(cli)
endif()

add_subdirectory(tests)
