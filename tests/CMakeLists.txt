cmake_minimum_required(VERSION 3.22)
project(VESCppTests)

include(${CMAKE_CURRENT_LIST_DIR}/cmake/vescpp_test.cmake)

add_test_target(NAME packet       
                SRCS  lib/packet.cpp lib/vesc_packets.cpp       
                LIBRARIES vescpp
              )
add_test_target(NAME all          
                SRCS  all.cpp                         
                ALL
              )
