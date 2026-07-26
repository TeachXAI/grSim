// Minimal example: custom policy loop without network or GUI.
// Build (optional): add to CMake or compile with -lgrsim -lode -lyaml-cpp -lpthread
#include "grsim/config.h"
#include "grsim/world.h"
#include "grsim/behaviors.h"
#include <iostream>

int main() {
    using namespace grsim;
    SimConfig cfg = SimConfig::defaults();
    cfg.robots_count = 2;
    cfg.logging.enabled = false;

    SimWorld world(cfg);
    CircleBehavior policy(cfg);

    for (int i = 0; i < 300; ++i) {
        if (i % 1 == 0) {  // every physics step ≈ 16ms
            auto vision = world.captureVision();
            auto cmds = policy.compute(vision, world.simTime());
            world.applyCommands(cmds);
        }
        world.step();
    }
    auto v = world.captureVision();
    std::cout << "Done. t=" << v.t_capture
              << " blue0=(" << v.robots_blue[0].x << "," << v.robots_blue[0].y << ")\n";
    return 0;
}
