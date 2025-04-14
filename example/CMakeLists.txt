cmake_minimum_required(VERSION 3.25)
project(vescpp_example)

include(FetchContent)
FetchContent_Declare(spdlog
  GIT_REPOSITORY  https://github.com/gabime/spdlog
  GIT_TAG         v1.10.0
)
FetchContent_Declare(lyra
  GIT_REPOSITORY  https://github.com/bfgroup/Lyra
  GIT_TAG         1.6.1
)

FetchContent_MakeAvailable(
  spdlog
  lyra
)

# VESCpp example
add_executable(${PROJECT_NAME}
  example.cpp
)

target_link_libraries(${PROJECT_NAME}
PRIVATE
  vescpp
  lyra
  spdlog
)