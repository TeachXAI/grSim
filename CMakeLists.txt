cmake_minimum_required(VERSION 3.14)
project(grSim LANGUAGES CXX)

# Headless, network-free, GUI-free simulation library (default).
# Legacy Qt GUI build is available with -DGRSIM_BUILD_LEGACY_GUI=ON
option(GRSIM_BUILD_LEGACY_GUI "Build the original Qt GUI grSim application" OFF)
option(GRSIM_BUILD_TESTS "Build unit tests" ON)
option(GRSIM_BUILD_APPS "Build runner applications" ON)

# Primary target: headless C++ library for ML/RL and in-process control
add_subdirectory(libgrsim)

if(GRSIM_BUILD_LEGACY_GUI)
  message(WARNING "Legacy GUI build is deprecated. Prefer the headless libgrsim target.")
  # Legacy sources remain under include/ and src/ for reference but are not
  # built by default. Network, Qt Widgets/OpenGL, VarTypes and protobuf vision
  # transport are intentionally excluded from the default product.
endif()
