cmake_minimum_required(VERSION 3.25)
project(vescpp)

include(FetchContent)
FetchContent_Declare(sockcanpp
  GIT_REPOSITORY  https://github.com/SimonCahill/libsockcanpp
  GIT_TAG         6548a99813262cf2d313a3ca6b7bd478af755e1c
)
FetchContent_Declare(spdlog
  GIT_REPOSITORY  https://github.com/gabime/spdlog
  GIT_TAG         v1.10.0
)
FetchContent_Declare(lyra
  GIT_REPOSITORY  https://github.com/bfgroup/Lyra
  GIT_TAG         1.6.1
)


FetchContent_MakeAvailable(
  sockcanpp
  spdlog
  lyra
)

# VescPP
add_library(${PROJECT_NAME} STATIC
  src/vescpp.cpp
  src/comm/can.cpp
)

target_include_directories(${PROJECT_NAME}
PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/include/
)

target_link_libraries(${PROJECT_NAME}
PUBLIC
  sockcanpp
  spdlog
  
)

set_target_properties(${PROJECT_NAME} 
PROPERTIES
  CXX_STANDARD 17
)

# VescPP example
add_executable(${PROJECT_NAME}_example
  src/example.cpp
)
target_link_libraries(${PROJECT_NAME}_example
PRIVATE
  ${PROJECT_NAME}
  lyra
)

add_subdirectory(tests)