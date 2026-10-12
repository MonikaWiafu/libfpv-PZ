cmake_minimum_required(VERSION 3.20)
project(libfpv VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

include(cmake/SimITLCore.cmake)

add_executable(simitl_smoke tests/simitl_smoke.cpp src/core/bf_log.c)
target_include_directories(simitl_smoke PRIVATE src/core)
target_link_libraries(simitl_smoke PRIVATE simitl_core pthread)
target_link_options(simitl_smoke PRIVATE -T${SIMITL_LINKER_SCRIPT})
