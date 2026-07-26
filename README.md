# grSim (headless library edition)

RoboCup Small Size League simulator, reworked as a **network-free, GUI-free C++ library** for fast in-process control and ML/RL training.

> Full conversion report: [docs/HEADLESS_REPORT.md](docs/HEADLESS_REPORT.md)

## Quick start (headless)

### Dependencies

- CMake 3.14+, C++17
- [ODE](http://www.ode.org) (`libode-dev`)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) (`libyaml-cpp-dev`)
- Optional: GoogleTest, Python3+Pillow+ffmpeg for tests/visualization

### Build & test

```bash
cmake -S libgrsim -B build_headless -DGRSIM_BUILD_TESTS=ON
cmake --build build_headless -j
ctest --test-dir build_headless --output-on-failure
```

### Run a demo (circle / square, sync or async)

```bash
./build_headless/grsim_run \
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

Example outputs are under `output/videos/` (`circle_sync.gif`, `square_sync.gif`, `circle_async.gif`).

## What changed vs classic grSim

| Classic grSim | This edition |
|---------------|--------------|
| Qt OpenGL GUI | Headless library |
| UDP vision + command ports | In-process API |
| VarTypes XML + robot `.ini` | YAML (`config/default.yaml`, `config/robots/`) |
| External team clients | Built-in `ClientController` + circle/square demos |
| Real-time GUI loop | Sync / async runner (~5–7× realtime) |

## Library layout

```
libgrsim/include/grsim/   # Public headers
libgrsim/src/             # Implementation
config/                   # YAML configuration
output/logs/              # CSV vision + command logs
output/videos/            # GIF/MP4 renders
```

## ML / RL usage sketch

```cpp
#include "grsim/config.h"
#include "grsim/runner.h"

auto cfg = grsim::SimConfig::loadFromFile("config/default.yaml");
cfg.logging.enabled = false;
grsim::SimulationRunner runner(cfg, grsim::createBehavior(cfg));

for (int step = 0; step < horizon; ++step) {
    auto obs = runner.world().captureVision();
    auto actions = policy(obs);          // your agent
    runner.world().applyCommands(actions);
    runner.world().step();
}
```

## Configuration

- World / client / logging: [`config/default.yaml`](config/default.yaml)
- Robot geometry & physics: [`config/robots/parsian.yaml`](config/robots/parsian.yaml)

## License

[GNU GPL v3](LICENSE.md) — same as upstream grSim.

## Upstream project

Based on [RoboCup-SSL/grSim](https://github.com/RoboCup-SSL/grSim). Classic authors and citation info are preserved in [AUTHORS.md](AUTHORS.md).
