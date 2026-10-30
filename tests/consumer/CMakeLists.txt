cmake_minimum_required(VERSION 3.16)

project(unified_state_install_consumer LANGUAGES CXX)

find_package(unified_state CONFIG REQUIRED)

add_executable(
  unified_state_install_consumer
  ../../examples/basic.cpp
)
target_link_libraries(
  unified_state_install_consumer
  PRIVATE
    unified_state::unified_state
)
