cmake_minimum_required(VERSION 3.22)
project(vescpp_example)

find_package(spdlog REQUIRED)
find_package(lyra REQUIRED)

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