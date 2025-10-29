cmake_minimum_required(VERSION 3.22)
project(vescpp_example)

if(NOT TARGET lyra)
  find_package(spdlog REQUIRED)
endif()
if(NOT TARGET lyra)
  find_package(lyra REQUIRED)
endif()

# VESCpp example
add_executable(${PROJECT_NAME}
  example.cpp
)

target_link_libraries(${PROJECT_NAME}
PRIVATE
  vescpp
  spdlog::spdlog
  lyra
)