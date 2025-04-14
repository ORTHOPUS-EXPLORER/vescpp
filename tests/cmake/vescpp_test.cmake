cmake_minimum_required(VERSION 3.22)

if(NOT TARGET Catch2)
  find_package(Catch2 3.8.0 QUIET)
  if(NOT Catch2_FOUND)
    include(FetchContent)
    FetchContent_Declare(Catch2
      GIT_REPOSITORY  https://github.com/catchorg/Catch2
      GIT_TAG         v3.8.0
      GIT_PROGRESS TRUE  GIT_SHALLOW TRUE
      FIND_PACKAGE_ARGS
    )
    FetchContent_MakeAvailable(Catch2)
    message(STATUS "[+] Use FetchContent to get Catch2 v3.8.0")
  else()
    message(STATUS "[+] Use System-provided spdlog ${spdlog_VERSION}")
  endif()
endif()

function(add_test_target)

  cmake_parse_arguments(
      ARG    # prefix
      "ALL"  # boolean arguments
      "NAME" # mono-valued arguments
      "SRCS;INCLUDE_DIRS;LIBRARIES;DEFINES" # multi-valued arguments
      ${ARGN} 
  )
  message(STATUS "[+] Add Test ${ARG_NAME}")

  if(${ARG_ALL})
    set(ARG_SRCS ${ARG_SRCS} ${all_srcs})
    set(ARG_DEFINES COMMON_MAIN ${ARG_DEFINES} ${all_defs})
    set(ARG_INCLUDE_DIRS ${ARG_INCLUDE_DIRS} ${all_incs})
    set(ARG_LIBRARIES ${ARG_LIBRARIES} ${all_libs})
  else()
    set(all_srcs ${all_srcs} ${ARG_SRCS} PARENT_SCOPE)
    set(all_defs ${all_defs} ${ARG_DEFINES} PARENT_SCOPE)
    set(all_incs ${all_incs} ${ARG_INCLUDE_DIRS} PARENT_SCOPE)
    set(all_libs ${all_libs} ${ARG_LIBRARIES} PARENT_SCOPE)
    set(all_tests ${all_tests} test_${ARG_NAME} PARENT_SCOPE)
  endif()

  add_executable(test_${ARG_NAME}
    ${ARG_SRCS}
  )

  target_compile_definitions(test_${ARG_NAME}
  PUBLIC
    ${ARG_DEFINES}
  )

  target_include_directories(test_${ARG_NAME}
  PUBLIC
      $<BUILD_INTERFACE:
          $<TARGET_PROPERTY:vescpp,INTERFACE_INCLUDE_DIRECTORIES>
          ${ARG_INCLUDE_DIRS}
      >
  )

  target_link_libraries(test_${ARG_NAME}
  PUBLIC
    ${ARG_LIBRARIES}
    vescpp
    Catch2WithMain
  )

  set_target_properties(test_${ARG_NAME}
  PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests
    COMPILE_FLAGS -fPIC
  )

  if(NOT ${ARG_ALL})
    add_test(run_test_${ARG_NAME}  ${CMAKE_BINARY_DIR}/tests/test_${ARG_NAME} )
  endif()

  add_custom_target(run_test_${ARG_NAME}
    COMMAND ${CMAKE_BINARY_DIR}/tests/test_${ARG_NAME} -r compact #-s
    DEPENDS test_${ARG_NAME}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  )
endfunction(add_test_target)
