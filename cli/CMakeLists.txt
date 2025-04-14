cmake_minimum_required(VERSION 3.25)
project(vescpp_cli)

include(FetchContent)
FetchContent_Declare(spdlog
  GIT_REPOSITORY  https://github.com/gabime/spdlog
  GIT_TAG         v1.10.0
  GIT_PROGRESS TRUE  GIT_SHALLOW TRUE
  FIND_PACKAGE_ARGS
)
FetchContent_Declare(lyra
  GIT_REPOSITORY  https://github.com/bfgroup/Lyra
  GIT_TAG         1.6.1
  GIT_PROGRESS TRUE  GIT_SHALLOW TRUE
  FIND_PACKAGE_ARGS
)
FetchContent_Declare(nlohmann_json
  GIT_REPOSITORY  https://github.com/nlohmann/json
  GIT_TAG         v3.11.3
  GIT_PROGRESS TRUE  GIT_SHALLOW TRUE
  FIND_PACKAGE_ARGS
)

FetchContent_MakeAvailable(
  spdlog
  lyra
  nlohmann_json
)

# VESCpp example
add_executable(${PROJECT_NAME}
  cli.cpp
)

target_link_libraries(${PROJECT_NAME}
PRIVATE
  vescpp
  lyra
  spdlog
  nlohmann_json::nlohmann_json
)