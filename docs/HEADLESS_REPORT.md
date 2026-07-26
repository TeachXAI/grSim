# grSim Headless Library — Implementation Report

**Date:** 2026-07-26  
**Goal:** Production-ready, network-free, GUI-free C++ simulation library for ML/RL training with an in-process Env API, hierarchical YAML config, logging, and 2D visualization.

---

## Summary

The default product is a **headless C++ static library** (`libgrsim`) plus a CLI runner (`grsim_run`) and a Gym-style **`Env`** adapter. Simulation, vision sampling, and robot control all happen **in-process** — there is **no UDP, no SSL-Vision multicast, no command sockets, no Qt Widgets/OpenGL GUI, and no VarTypes/XML**.

| Concern | Before (classic grSim) | After |
|--------|------------------------|-------|
| GUI | Qt Widgets + OpenGL | Removed |
| Config | VarTypes XML + `.ini` robots | Hierarchical YAML |
| Vision / commands | Network (protobuf UDP) | In-process `VisionFrame` / `RobotCommand` |
| Clients | External Qt/Java samples | Built-in behaviours + `ClientController` + `Env` |
| Control timing | External wall-clock packets | Sync or async, every 16 ms sim-time |
| ML loop | Not supported cleanly | `Env::reset` / `Env::step` + `SimulationRunner` |
| Visualization | 3D OpenGL window | Offline 2D top-down GIF/MP4 from logs |

Legacy Qt/network sources, clients, textures, protobuf modules, and `.ini` robot files have been **removed** from the tree. The default build depends only on **ODE + yaml-cpp + pthread**.

---

## Architecture

```
libgrsim/
  include/grsim/     Public API (env, runner, world, config, behaviors, ...)
  src/               Simulation, physics, clients, logging, env
  apps/grsim_run.cpp CLI entry point
  apps/example_ml_loop.cpp
  tools/visualize_logs.py
  tests/             GoogleTest suite (43 tests)

config/
  default.yaml       Hierarchical world + control + env + logging
  robots/parsian.yaml
```

### Core types

- **`SimConfig`** — loads hierarchical YAML (`simulation`, `field`, `ball`, `physics`, `robots`, `control`, `env`, `logging`)
- **`SimWorld`** — ODE physics world; `step()`, `applyCommands()`, `captureVision()`
- **`ClientController`** — pure virtual `compute(vision, t) → commands`
- **`CircleBehavior` / `SquareBehavior` / `IdleBehavior`**
- **`SimulationRunner`** — sync/async loops + logging
- **`Env`** — Gym-style adapter: `reset`, `step`, `observation`, rewards, termination
- **`SimLogger`** — CSV vision + command logs

### Control modes

1. **Sync** — single thread; every `control_period_ms` of *sim time*, client runs and commands are applied, then physics steps.
2. **Async** — client on a background thread, paced by **simulation time**; commands enqueued under a mutex and drained at each physics step.

### Env (ML/RL)

```cpp
#include "grsim/env.h"

grsim::Env env(grsim::SimConfig::loadFromFile("config/default.yaml"));
auto obs = env.reset(42);
while (!env.done()) {
    auto actions = policy(obs.vision);
    auto result = env.step(actions);
    // result.observation, result.reward, result.terminated, result.truncated
    obs = result.observation;
}
// Full access still available:
env.world(); env.runner();
```

All Env parameters (`max_episode_time`, `reward_type`, `terminate_on_goal`, `physics_steps_per_action`, `seed`) come from YAML `env:`.

---

## Build & run

### Dependencies

- C++17, CMake ≥ 3.14  
- `libode-dev`, `libyaml-cpp-dev`  
- (tests) GoogleTest auto-fetched  
- (viz) Python3 + Pillow + ffmpeg  

### Build

```bash
cmake -S . -B build -DGRSIM_BUILD_TESTS=ON
cmake --build build -j
ctest --test-dir build/libgrsim --output-on-failure
```

### Demos

```bash
./build/libgrsim/grsim_run --config config/default.yaml \
  --duration 10 --mode sync --behavior circle --log-dir output/logs
```

Modes: `sync|async`. Behaviours: `circle|square|idle`.

---

## Validation (2026-07-26)

### Tests

```
43/43 tests passed
```

Coverage: YAML config (defaults, hierarchical, legacy keys, invalid), physics, circle/square/idle, sync+async runs, Env API (create/reset/step/obs/time/termination/seed determinism), regression path quality, logger, SimulationRunner compatibility.

### Demo metrics (10 s, 3 robots/team)

| Run | Realtime | Circle mean radius / Square path |
|-----|----------|-----------------------------------|
| circle_sync | ~8.5× | ~1.43 m (target 1.5), std ≤ 0.22 |
| circle_async | ~8.0× | matches sync |
| square_sync | ~4.0× | path ~6.7 m, bbox ~2×2 m |
| square_async | ~3.9× | matches sync |

Visual inspection of GIF frames confirms circular trails (circle) and rectangular paths (square) in both sync and async modes, with command arrows and fading trails.

Artifacts under `output/` (gitignored): logs, GIFs, MP4s.

---

## Configuration hierarchy

```yaml
simulation: { division, robots_count, formation, seed }
field: { A: {...}, B: {...} }
ball: {...}
physics: {...}
robots: { blue: parsian, yellow: parsian }
control: { mode, behavior, control_period_ms, speed, circle_radius, square_size, ... }
env: { max_episode_time, reward_type, terminate_on_goal, physics_steps_per_action, seed }
logging: { enabled, directory, prefix, log_vision, log_commands }
```

Legacy keys (`client:`, `teams:`, top-level `division`) remain accepted.

---

## Conclusion

grSim is usable as a **fast, headless, in-process C++ library** for ML/RL:

- No network path for vision or commands  
- No Qt / VarTypes / protobuf in the default product  
- Hierarchical YAML for simulation, robots, control, env, logging  
- Sync and async 16 ms client control  
- Standard Env API for training loops  
- Logs + GIF/MP4 verification of circle and square motions  
- **~4–8× real-time** with 6 robots  
- **43/43 unit tests passing**
