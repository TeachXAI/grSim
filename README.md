# grSim (headless library edition)

RoboCup Small Size League simulator, reworked as a **network-free, GUI-free C++ library** for fast in-process control and ML/RL training.

> Full conversion report: [docs/HEADLESS_REPORT.md](docs/HEADLESS_REPORT.md)

## Quick start

### Dependencies

- CMake 3.14+, C++17
- [ODE](http://www.ode.org) (`libode-dev`)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) (`libyaml-cpp-dev`)
- Optional: GoogleTest (auto-fetched), Python3+Pillow+ffmpeg for visualization

```bash
./install_deps.sh --skip-install
# or manually:
# sudo apt install build-essential cmake pkg-config libode-dev libyaml-cpp-dev
```

### Build & test

```bash
cmake -S . -B build -DGRSIM_BUILD_TESTS=ON
cmake --build build -j
ctest --test-dir build/libgrsim --output-on-failure
# or: make && make test
```

### Run a demo (circle / square, sync or async)

```bash
./build/libgrsim/grsim_run \
  --config config/default.yaml \
  --duration 8 \
  --mode sync \
  --behavior circle \
  --robots 3 \
  --log-dir output/logs
```

### Visualize logs (top-down 2D GIF/MP4)

```bash
python3 libgrsim/tools/visualize_logs.py \
  --vision output/logs/<run>_vision.csv \
  --commands output/logs/<run>_commands.csv \
  --meta output/logs/<run>_meta.txt \
  --out output/videos/run.gif
```

## Library layout

```
libgrsim/include/grsim/   # Public headers (env, runner, world, config, ...)
libgrsim/src/             # Implementation
libgrsim/apps/            # grsim_run CLI + examples
libgrsim/tests/           # GoogleTest suite
config/                   # Hierarchical YAML configuration
```

## ML / RL Env API

```cpp
#include "grsim/env.h"

auto cfg = grsim::SimConfig::loadFromFile("config/default.yaml");
cfg.logging.enabled = false;
grsim::Env env(cfg);

auto obs = env.reset(/*seed=*/42);
for (int t = 0; t < horizon; ++t) {
    std::vector<grsim::RobotCommand> actions = policy(obs.vision);
    auto result = env.step(actions);
    // result.observation, result.reward, result.terminated, result.truncated
    obs = result.observation;
    if (result.done()) break;
}

// Underlying SimulationRunner / SimWorld remain fully accessible:
env.world().setBallPose(0, 0);
env.runner().run(10.0);  // full-duration circle/square demo
```

## Configuration

All parameters live in the hierarchical YAML tree ([`config/default.yaml`](config/default.yaml)):

| Section | Purpose |
|---------|---------|
| `simulation` | division, robots_count, formation, seed |
| `field` / `ball` / `physics` | world geometry and ODE settings |
| `robots` | team robot packs (`config/robots/*.yaml`) |
| `control` | sync/async mode, behaviours, control period |
| `env` | max episode time, reward type, terminate_on_goal, steps/action |
| `logging` | CSV vision/command logs |

Legacy keys (`client:`, `teams:`, top-level `division`) are still accepted.

## License

[GNU GPL v3](LICENSE.md) — same as upstream grSim.

## Upstream project

Based on [RoboCup-SSL/grSim](https://github.com/RoboCup-SSL/grSim). Classic authors and citation info are preserved in [AUTHORS.md](AUTHORS.md).
