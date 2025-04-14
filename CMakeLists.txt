cmake_minimum_required(VERSION 3.22)
project(vescpp)

option(VESCPP_BUILD_EXAMPLE "Build example application" ON)
option(VESCPP_BUILD_TESTS   "Build unittests"           ON)

include(FetchContent)
set(EXT_PKGS)
if(NOT TARGET spdlog::spdlog)
  FetchContent_Declare(spdlog
    GIT_REPOSITORY  https://github.com/gabime/spdlog
    GIT_TAG         v1.10.0
    GIT_PROGRESS TRUE  GIT_SHALLOW TRUE
    FIND_PACKAGE_ARGS 1.10.0
  )
  FetchContent_MakeAvailable(spdlog)
  if(NOT spdlog_FOUND)
    message(STATUS "[+] Use FetchContent to get spdlog v1.10.0")
  else()
    message(STATUS "[+] Use System-provided spdlog ${spdlog_VERSION}")
  endif()
endif()
if(NOT TARGET nlohmann_json::nlohmann_json)
  FetchContent_Declare(nlohmann_json
    GIT_REPOSITORY  https://github.com/nlohmann/json
    GIT_TAG         v3.11.3
    GIT_PROGRESS TRUE  GIT_SHALLOW TRUE
    FIND_PACKAGE_ARGS 3.11.3
  )
  FetchContent_MakeAvailable(nlohmann_json)
  if(NOT nlohmann_json_FOUND)
    message(STATUS "[+] Use FetchContent to get nlohmann_json v3.11.3")
  else()
    message(STATUS "[+] Use System-provided nlohmann_json ${nlohmann_json_VERSION}")
  endif()
endif()
if(NOT TARGET heatshrink)
  include(ExternalProject)
  set(HEATSHRINK_SRC_DIR ${CMAKE_BINARY_DIR}/heatshrink)
  #set(HEATSHRINK_INSTALL_DIR ${HEATSHRINK_SRC_DIR}/install)
  ExternalProject_Add(heatshrink_build
      GIT_REPOSITORY https://github.com/atomicobject/heatshrink
      GIT_TAG        develop
      GIT_PROGRESS TRUE  GIT_SHALLOW TRUE
      UPDATE_COMMAND ""
      CONFIGURE_COMMAND ""
      SOURCE_DIR ${HEATSHRINK_SRC_DIR}
      BUILD_IN_SOURCE 1
      BUILD_COMMAND make libraries
      INSTALL_COMMAND ""
  )
  add_library(heatshrink STATIC IMPORTED)
  add_dependencies(heatshrink
    heatshrink_build
  )
  file(MAKE_DIRECTORY ${HEATSHRINK_SRC_DIR}/include)  # Required to please INTERFACE_INCLUDE_DIRECTORIES
  set_target_properties(heatshrink
      PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES ${HEATSHRINK_SRC_DIR}/include/
        IMPORTED_LOCATION ${HEATSHRINK_SRC_DIR}/build/libheatshrink_static.a
  )
  message(STATUS "[+] Use ExternalProject to get heatshrink develop")
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
  $<BUILD_INTERFACE:
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/src/3rdparty
  >
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
PUBLIC
  spdlog::spdlog
  nlohmann_json::nlohmann_json
  heatshrink
)

add_dependencies(${PROJECT_NAME}
  heatshrink
)

set_target_properties(${PROJECT_NAME}
PROPERTIES
  CXX_STANDARD 17
)

set_target_properties(${PROJECT_NAME}
PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

install(
  DIRECTORY   include/
  DESTINATION include
)

if(VESCPP_BUILD_EXAMPLE)
  # Lyra is used in all examples so fetch it here
  if(NOT TARGET lyra)
    FetchContent_Declare(lyra
      GIT_REPOSITORY  https://github.com/bfgroup/Lyra
      GIT_TAG         1.6.1
      GIT_PROGRESS TRUE  GIT_SHALLOW TRUE
      FIND_PACKAGE_ARGS 1.6.1
    )
    FetchContent_MakeAvailable(lyra)
    if(NOT lyra_FOUND)
      message(STATUS "[+] Use FetchContent to get lyra 1.6.1")
    else()
      message(STATUS "[+] Use System-provided lyra ${lyra_VERSION}")
    endif()
  endif()
  add_subdirectory(example)
  add_subdirectory(cli)
endif()

if(VESCPP_BUILD_TESTS)
  add_subdirectory(tests)
endif()
