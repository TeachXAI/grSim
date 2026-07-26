cmake_minimum_required(VERSION 3.14)
project(grSim LANGUAGES CXX)

# Headless, network-free, GUI-free simulation library (default product).
# Primary target lives in libgrsim/ and depends only on ODE + yaml-cpp.
option(GRSIM_BUILD_TESTS "Build unit tests" ON)
option(GRSIM_BUILD_APPS "Build runner applications" ON)

add_subdirectory(libgrsim)
