cmake_minimum_required(VERSION 3.14)
project(grsim_headless VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(GRSIM_BUILD_TESTS "Build unit tests" ON)
option(GRSIM_BUILD_APPS "Build runner apps" ON)

find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

# ODE: prefer plain library (Ubuntu ships libode-dev without reliable CMake package)
find_path(ODE_INCLUDE_DIR ode/ode.h)
find_library(ODE_LIBRARY NAMES ode)
if(NOT ODE_INCLUDE_DIR OR NOT ODE_LIBRARY)
  message(FATAL_ERROR "ODE not found. Install libode-dev")
endif()

set(GRSIM_LIB_SOURCES
  src/config.cpp
  src/robot.cpp
  src/world.cpp
  src/behaviors.cpp
  src/logger.cpp
  src/runner.cpp
  src/env.cpp
  src/physics/pobject.cpp
  src/physics/pworld.cpp
  src/physics/pball.cpp
  src/physics/pbox.cpp
  src/physics/pcylinder.cpp
  src/physics/pfixedbox.cpp
  src/physics/pground.cpp
)

add_library(grsim STATIC ${GRSIM_LIB_SOURCES})
add_library(grsim::grsim ALIAS grsim)

target_include_directories(grsim
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${ODE_INCLUDE_DIR}
)

target_link_libraries(grsim PUBLIC yaml-cpp ${ODE_LIBRARY} Threads::Threads)

# Project root (parent of libgrsim) for config files
get_filename_component(GRSIM_PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)

if(GRSIM_BUILD_APPS)
  add_executable(grsim_run apps/grsim_run.cpp)
  target_link_libraries(grsim_run PRIVATE grsim)
  add_custom_command(TARGET grsim_run POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:grsim_run>/config/robots
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${GRSIM_PROJECT_ROOT}/config/default.yaml
      $<TARGET_FILE_DIR:grsim_run>/config/default.yaml
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${GRSIM_PROJECT_ROOT}/config/robots/parsian.yaml
      $<TARGET_FILE_DIR:grsim_run>/config/robots/parsian.yaml
  )

  add_executable(example_ml_loop apps/example_ml_loop.cpp)
  target_link_libraries(example_ml_loop PRIVATE grsim)
endif()

if(GRSIM_BUILD_TESTS)
  enable_testing()
  find_package(GTest QUIET)
  if(NOT GTest_FOUND)
    include(FetchContent)
    FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
  endif()

  add_executable(grsim_tests
    tests/test_config.cpp
    tests/test_physics.cpp
    tests/test_behaviors.cpp
    tests/test_simulation.cpp
    tests/test_logger.cpp
    tests/test_env.cpp
    tests/test_regression_behaviors.cpp
  )
  target_link_libraries(grsim_tests PRIVATE grsim GTest::gtest_main)
  target_compile_definitions(grsim_tests PRIVATE
    GRSIM_SOURCE_DIR="${GRSIM_PROJECT_ROOT}"
  )
  include(GoogleTest)
  gtest_discover_tests(grsim_tests DISCOVERY_TIMEOUT 30)
endif()

install(TARGETS grsim ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/grsim DESTINATION include)
install(FILES ${GRSIM_PROJECT_ROOT}/config/default.yaml DESTINATION share/grsim/config)
install(DIRECTORY ${GRSIM_PROJECT_ROOT}/config/robots DESTINATION share/grsim/config)
if(GRSIM_BUILD_APPS)
  install(TARGETS grsim_run RUNTIME DESTINATION bin)
endif()
