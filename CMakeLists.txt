cmake_minimum_required(VERSION 3.16)

project(
  unified_state
  VERSION 0.1.0
  DESCRIPTION "Causal state-history and publication layer"
  LANGUAGES CXX
)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(Threads REQUIRED)

add_library(unified_state INTERFACE)
add_library(unified_state::unified_state ALIAS unified_state)

target_compile_features(unified_state INTERFACE cxx_std_17)
target_link_libraries(unified_state INTERFACE Threads::Threads)
target_include_directories(
  unified_state
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

option(
  UNIFIED_STATE_BUILD_BENCHMARKS
  "Build host performance benchmarks"
  OFF
)

option(
  UNIFIED_STATE_BUILD_EXAMPLES
  "Build standalone examples"
  OFF
)

include(CTest)
if(BUILD_TESTING)
  add_executable(
    unified_state_timeline_test
    tests/timeline_test.cpp
  )
  target_link_libraries(
    unified_state_timeline_test
    PRIVATE
      unified_state::unified_state
  )
  add_test(
    NAME unified_state_timeline_test
    COMMAND unified_state_timeline_test
  )
endif()

if(UNIFIED_STATE_BUILD_BENCHMARKS)
  add_executable(
    unified_state_timeline_bench
    benchmarks/timeline_bench.cpp
  )
  target_link_libraries(
    unified_state_timeline_bench
    PRIVATE
      unified_state::unified_state
  )
endif()

if(UNIFIED_STATE_BUILD_EXAMPLES)
  add_executable(
    unified_state_basic_example
    examples/basic.cpp
  )
  target_link_libraries(
    unified_state_basic_example
    PRIVATE
      unified_state::unified_state
  )
endif()

install(
  TARGETS unified_state
  EXPORT unified_stateTargets
)
install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING
    PATTERN "*.hpp"
)
install(
  EXPORT unified_stateTargets
  FILE unified_stateTargets.cmake
  NAMESPACE unified_state::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/unified_state
)

configure_package_config_file(
  cmake/unified_stateConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/unified_stateConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/unified_state
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/unified_stateConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/unified_stateConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/unified_stateConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/unified_state
)
