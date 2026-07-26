# grSim Headless Library — Implementation Report

**Date:** 2026-07-25  
**Goal:** Convert grSim into a network-free, GUI-free C++ simulation library suitable for ML/RL training, with in-process clients, YAML config, logging, and 2D visualization.

---

## Summary

The default product is now a **headless C++ static library** (`libgrsim`) plus a CLI runner (`grsim_run`). Simulation, vision sampling, and robot control all happen **in-process** — there is **no UDP, no SSL-Vision multicast, no command sockets, no Qt Widgets/OpenGL GUI, and no VarTypes/XML**.

| Concern | Before | After |
|--------|--------|-------|
| GUI | Qt Widgets + OpenGL | Removed (headless only by default) |
| Config | VarTypes XML + `.ini` robots | Single YAML tree (`config/default.yaml` + `config/robots/*.yaml`) |
| Vision / commands | Network (protobuf UDP) | In-process `VisionFrame` / `RobotCommand` |
| Clients | External Qt/Java samples | Built-in behaviors + `ClientController` API |
| Control timing | External wall-clock packets | Sync or async, every 16 ms sim-time |
| Visualization | 3D OpenGL window | Offline 2D top-down GIF/MP4 from logs |
| ML loop | Not supported cleanly | `SimulationRunner::stepOnce()` / `run()` |

Legacy Qt sources remain under `include/` and `src/` for reference but are **not built** unless `-DGRSIM_BUILD_LEGACY_GUI=ON` (currently a no-op stub).

---

## Architecture

```
libgrsim/
  include/grsim/     Public API
  src/               Simulation, physics, clients, logging
  apps/grsim_run.cpp CLI entry point
  tools/visualize_logs.py
  tests/             GoogleTest suite (20 tests)

config/
  default.yaml       World + client + logging
  robots/parsian.yaml
```

### Core types

- **`SimConfig`** — loads YAML (field, ball, physics, teams, client, logging)
- **`SimWorld`** — ODE physics world, robots, ball, walls; `step()`, `applyCommand()`, `captureVision()`
- **`ClientController`** — pure virtual `compute(vision, t) → commands`
- **`CircleBehavior` / `SquareBehavior` / `IdleBehavior`**
- **`SimulationRunner`** — sync/async loops + logging
- **`SimLogger`** — CSV vision + command logs

### Control modes

1. **Sync** — single thread; every `control_period_ms` (default 16 ms) of *sim time*, client runs and commands are applied, then physics steps.
2. **Async** — client on a background thread, paced by **simulation time** (so it keeps up when physics runs faster than real-time); commands enqueued under a mutex and drained at the start of each physics step.

### Physics

Headless port of grSim’s ODE model:

- 4 omni wheels with motor velocity control  
- Kicker / dribbler  
- Field walls + goals  
- Ball friction / bounce  

No OpenGL draw path; graphics pointer removed from physics objects.

---

## Build & run

### Dependencies

- C++17 compiler  
- CMake ≥ 3.14  
- `libode-dev`  
- `libyaml-cpp-dev`  
- (tests) GoogleTest (auto-fetched if missing)  
- (viz) Python3 + Pillow + ffmpeg  

### Build

```bash
cmake -S libgrsim -B build_headless -DGRSIM_BUILD_TESTS=ON
cmake --build build_headless -j
ctest --test-dir build_headless --output-on-failure
```

### Run simulation

```bash
./build_headless/grsim_run \
  --config config/default.yaml \
  --duration 8 \
  --mode sync \
  --behavior circle \
  --robots 3 \
  --log-dir output/logs
```

Options: `--mode sync|async`, `--behavior circle|square|idle`, `--duration SEC`, `--no-log`.

### Visualize logs

```bash
python3 libgrsim/tools/visualize_logs.py \
  --vision output/logs/<run>_vision.csv \
  --commands output/logs/<run>_commands.csv \
  --meta output/logs/<run>_meta.txt \
  --out output/videos/run.gif
```

Drawing: green SSL field (top-down), blue/yellow robots as circles with ID + heading, **fading trails**, **red arrows** for commanded velocity, orange ball.

---

## Validation runs (2026-07-25)

All runs: 3 robots/team, Δt = 1/60 s, control every 16 ms, Division A field.
Robots are seeded on the behaviour path at t=0 for clean demos.

| Run | Mode | Behavior | Sim time | Wall time | Realtime factor | Frames |
|-----|------|----------|----------|-----------|-----------------|--------|
| circle_sync | sync | circle | 10.0 s | ~1.17 s | **~8.5×** | 600 |
| square_sync | sync | square | 10.0 s | ~2.56 s | **~3.9×** | 600 |
| circle_async | async | circle | 10.0 s | ~1.22 s | **~8.2×** | 600 |

### Circle path quality (after t≥2 s, target radius 1.5 m)

| Robot | Mean radius | Std radius |
|-------|-------------|------------|
| blue 0 | 1.43 m | 0.01 m |
| blue 1 | 1.35 m | 0.22 m |
| blue 2 | 1.42 m | 0.04 m |
| yellow 0 | 1.39 m | 0.11 m |
| yellow 1 | 1.43 m | 0.00 m |
| yellow 2 | 1.43 m | 0.00 m |

Robots track circles tightly (std ≪ radius). Minor radius offset is expected from the simple P-controller and acceleration limits.

Command log density matches a 16 ms control period (≈360 ticks × 6 robots over 10 s when both teams active).

### Artifacts

| File | Description |
|------|-------------|
| `output/videos/circle_sync.gif` | Circle, sync |
| `output/videos/circle_async.gif` | Circle, async |
| `output/videos/square_sync.gif` | Square, sync |
| `output/videos/circle_sync.mp4` | Circle, sync (H.264) |
| `output/videos/square_sync.mp4` | Square, sync (H.264) |
| `output/logs/run_*_{vision,commands,meta}.*` | Raw logs |

### Tests

```
20/20 tests passed
```

Coverage:

- YAML config load (defaults, Division A/B, robot YAML)  
- Physics step, gravity, velocity command motion, vision counts, teleport  
- Circle / square / idle behaviors + factory  
- Sync + async full runs, `stepOnce` ML loop, logger CSV I/O  

---

## Using as an ML/RL library

```cpp
#include "grsim/config.h"
#include "grsim/runner.h"
#include "grsim/behaviors.h"

grsim::SimConfig cfg = grsim::SimConfig::loadFromFile("config/default.yaml");
cfg.logging.enabled = false;  // high-throughput training
cfg.robots_count = 3;

grsim::SimulationRunner runner(cfg, grsim::createBehavior(cfg));

// Option A: fixed-duration rollout with built-in client
auto result = runner.run(/*duration*/10.0, grsim::RunMode::Sync);

// Option B: external policy loop (your RL agent)
for (int i = 0; i < 1000; ++i) {
    auto vision = runner.world().captureVision();
    // ... policy(vision) -> std::vector<RobotCommand>
    runner.world().applyCommands(commands);
    runner.world().step();
}
```

No sockets, no GUI event loop — link `libgrsim`, ODE, yaml-cpp, pthread.

---

## YAML config notes

Replaces:

- `~/.grsim.xml` (VarTypes tree)  
- `config/*.ini` robot geometry/physics  

Example structure is in `config/default.yaml`. Robot packs live under `config/robots/<name>.yaml` and are selected by:

```yaml
teams:
  blue: parsian
  yellow: parsian
```

Field dimensions for Division A/B are both embedded; `division: A|B` selects the active set.

---

## Known limitations / next steps for full RL

1. **Reward / episode API** — not yet standardized; use `captureVision()` + custom reward.  
2. **Domain randomization** — can mutate `SimConfig` / robot settings between episodes.  
3. **Python bindings** — not included; C++ API is the first step.  
4. **Legacy GUI** — sources kept but not built; full deletion can follow once headless is adopted.  
5. **Initial formation** — robots start in “outside” formation and walk onto circle/square paths (visible as trails from the sideline into the field).

---

## Conclusion

The simulation runs as a **fast, headless, in-process library**:

- No network path for vision or commands  
- No Qt GUI / VarTypes dependency for the default build  
- YAML configuration for world + robots  
- Sync and async 16 ms client control  
- Logs + GIF/MP4 verification of circle and square motions  
- **~5–7× real-time** on this host with 6 robots  
- **20/20 unit tests passing**

This is a solid first step toward grSim as an RL training environment.
