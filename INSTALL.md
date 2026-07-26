# grSim — Install (headless library)

## Overview

The default product is a **headless C++ simulation library** (`libgrsim`) for
in-process ML/RL and control. It does **not** require Qt, OpenGL, protobuf,
VarTypes, or network ports.

## Dependencies

- CMake ≥ 3.14
- C++17 compiler
- [Open Dynamics Engine (ODE)](http://www.ode.org) — double precision
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
- (tests) GoogleTest — fetched automatically by CMake if missing
- (optional viz) Python 3, Pillow, ffmpeg

### Ubuntu / Debian

```bash
sudo apt install build-essential cmake pkg-config libode-dev libyaml-cpp-dev
# optional:
sudo apt install python3 python3-numpy python3-pil ffmpeg
```

### Arch Linux

```bash
sudo pacman -S base-devel cmake ode yaml-cpp
```

### macOS

```bash
brew install cmake pkg-config
brew tap robotology/formulae
brew install robotology/formulae/ode yaml-cpp
```

## Build

```bash
# One-shot installer (deps + build + optional install)
./install_deps.sh --skip-install

# Or manually:
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DGRSIM_BUILD_TESTS=ON
cmake --build build -j
ctest --test-dir build/libgrsim --output-on-failure
cmake --install build   # optional, default prefix /usr/local
```

```bash
make          # configure + build
make test     # run unit tests
make demos    # circle/square sync+async runs with logs
```

## Docker

```bash
docker build -t grsim-headless .
docker run --rm grsim-headless
# override:
docker run --rm grsim-headless --duration 10 --behavior square --mode async --no-log
```

## Using the library in your project

```cmake
find_package(yaml-cpp REQUIRED)
find_library(ODE_LIBRARY ode)
add_subdirectory(path/to/grSim/libgrsim)  # or link installed libgrsim
target_link_libraries(my_agent PRIVATE grsim)
```

```cpp
#include "grsim/env.h"
```

Link with: `grsim`, `ode`, `yaml-cpp`, `pthread`.
