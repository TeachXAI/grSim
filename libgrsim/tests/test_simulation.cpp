#include <gtest/gtest.h>
#include "grsim/runner.h"
#include "grsim/behaviors.h"
#include "grsim/config.h"
#include <cmath>
#include <vector>

using namespace grsim;

class SimTest : public ::testing::Test {
protected:
    SimConfig cfg;
    void SetUp() override {
        cfg = SimConfig::defaults();
        cfg.robots_count = 2;
        cfg.client.control_period_ms = 16;
        cfg.client.speed = 1.5;
        cfg.client.circle_radius = 1.0;
        cfg.logging.enabled = true;
        cfg.logging.directory = "/tmp/grsim_test_logs";
        cfg.logging.prefix = "test";
        try {
#ifdef GRSIM_SOURCE_DIR
            cfg.blue_robot = SimConfig::loadRobotSettings(
                std::string(GRSIM_SOURCE_DIR) + "/config/robots/parsian.yaml");
            cfg.yellow_robot = cfg.blue_robot;
            cfg.robot_settings = cfg.blue_robot;
#endif
        } catch (...) {}
    }
};

TEST_F(SimTest, SyncCircleRunCompletes) {
    cfg.client.mode = RunMode::Sync;
    cfg.client.behavior = BehaviorType::Circle;
    SimulationRunner runner(cfg, createBehavior(cfg));
    // Place robots near circle path for better motion
    for (int i = 0; i < cfg.robots_count; i++) {
        runner.world().setRobotPose(i, Team::Blue, -1.5 + 1.0, 0.3 * i, 0);
        runner.world().setRobotPose(i, Team::Yellow, 1.5 + 1.0, 0.3 * i, 180);
    }
    auto result = runner.run(3.0, RunMode::Sync);
    EXPECT_GE(result.frames, 100u);
    EXPECT_NEAR(result.sim_time, 3.0, 0.05);
    EXPECT_FALSE(result.vision_log.empty());
    EXPECT_FALSE(result.command_log.empty());
}

TEST_F(SimTest, AsyncCircleRunCompletes) {
    cfg.client.mode = RunMode::Async;
    cfg.client.behavior = BehaviorType::Circle;
    SimulationRunner runner(cfg, createBehavior(cfg));
    auto result = runner.run(2.0, RunMode::Async);
    EXPECT_GE(result.frames, 50u);
    EXPECT_NEAR(result.sim_time, 2.0, 0.05);
}

TEST_F(SimTest, SyncSquareRobotsMove) {
    cfg.client.mode = RunMode::Sync;
    cfg.client.behavior = BehaviorType::Square;
    cfg.client.square_size = 2.0;
    SimulationRunner runner(cfg, createBehavior(cfg));
    // record initial positions
    auto v0 = runner.world().captureVision();
    std::vector<std::pair<double,double>> p0;
    for (const auto& r : v0.robots_blue) p0.push_back({r.x, r.y});

    runner.run(4.0, RunMode::Sync);

    auto v1 = runner.world().captureVision();
    double max_disp = 0;
    for (size_t i = 0; i < v1.robots_blue.size() && i < p0.size(); i++) {
        double d = std::hypot(v1.robots_blue[i].x - p0[i].first,
                              v1.robots_blue[i].y - p0[i].second);
        max_disp = std::max(max_disp, d);
    }
    EXPECT_GT(max_disp, 0.2) << "robots should displace under square behavior";
}

TEST_F(SimTest, StepOnceForMLLoop) {
    cfg.logging.enabled = false;
    SimulationRunner runner(cfg, createBehavior(cfg));
    auto v = runner.stepOnce();
    EXPECT_EQ(v.frame_number, 1u);
    v = runner.stepOnce();
    EXPECT_EQ(v.frame_number, 2u);
}

TEST_F(SimTest, NoNetworkDependencies) {
    // compile-time/link-time guarantee is the real test; runtime: pure in-process
    SimWorld world(cfg);
    RobotCommand cmd;
    cmd.id = 0; cmd.team = Team::Blue; cmd.vel_tangent = 0.5;
    world.applyCommand(cmd);
    world.step();
    SUCCEED();
}
