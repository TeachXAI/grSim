// Minimal example: Gym-style Env loop without network or GUI.
#include "grsim/env.h"
#include "grsim/behaviors.h"
#include <iostream>

int main() {
    using namespace grsim;

    SimConfig cfg = SimConfig::defaults();
    cfg.robots_count = 2;
    cfg.logging.enabled = false;
    cfg.env.max_episode_time = 5.0;
    cfg.env.reward_type = "zero";
    cfg.client.behavior = BehaviorType::Circle;

    Env env(cfg);
    auto obs = env.reset(42);

    CircleBehavior policy(cfg);
    double total_reward = 0.0;
    int steps = 0;
    while (!env.done() && steps < 300) {
        auto actions = policy.compute(obs.vision, obs.sim_time);
        auto result = env.step(actions);
        total_reward += result.reward;
        obs = result.observation;
        ++steps;
    }

    std::cout << "Done. steps=" << steps
              << " t=" << obs.sim_time
              << " reward=" << total_reward
              << " blue0=(" << obs.vision.robots_blue[0].x
              << "," << obs.vision.robots_blue[0].y << ")\n";
    return 0;
}
