cmake_minimum_required(VERSION 3.25)
project(vescpp)

option(VESCPP_BUILD_EXAMPLE "Build example application" ON)
option(VESCPP_BUILD_TESTS   "Build unittests"           ON)

include(FetchContent)
FetchContent_Declare(spdlog
  GIT_REPOSITORY  https://github.com/gabime/spdlog
  GIT_TAG         v1.10.0
)

FetchContent_MakeAvailable(
  spdlog
)

# VESCpp
add_library(${PROJECT_NAME} STATIC
  src/vescpp.cpp
  src/comm/can.cpp
  src/vesc/utils.cpp
  src/packet/vesc_packets/00_fwversion.cpp
)

target_include_directories(${PROJECT_NAME}
PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/include/
)

target_link_libraries(${PROJECT_NAME}
PUBLIC
  spdlog
)

set_target_properties(${PROJECT_NAME} 
PROPERTIES
  CXX_STANDARD 17
)

if(VESCPP_BUILD_EXAMPLE)
  add_subdirectory(example)
endif()

if(VESCPP_BUILD_TESTS)
  add_subdirectory(tests)
endif()