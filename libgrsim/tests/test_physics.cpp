#include <gtest/gtest.h>
#include "grsim/world.h"
#include "grsim/config.h"
#include <cmath>

using namespace grsim;

class PhysicsTest : public ::testing::Test {
protected:
    SimConfig cfg;
    void SetUp() override {
        cfg = SimConfig::defaults();
        cfg.robots_count = 2;
        cfg.logging.enabled = false;
        // load robot settings from project if possible
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

TEST_F(PhysicsTest, WorldConstructsAndSteps) {
    SimWorld world(cfg);
    EXPECT_NEAR(world.simTime(), 0.0, 1e-9);
    world.step();
    EXPECT_GT(world.simTime(), 0.0);
    EXPECT_EQ(world.frameNumber(), 1u);
}

TEST_F(PhysicsTest, BallFallsUnderGravity) {
    SimWorld world(cfg);
    world.setBallPose(0, 0, 1.0, 0, 0, 0);
    auto b0 = world.ballState();
    EXPECT_NEAR(b0.z, 1.0, 0.05);
    for (int i = 0; i < 120; i++) world.step();  // 2 seconds
    auto b1 = world.ballState();
    // ball should be near ground
    EXPECT_LT(b1.z, 0.2);
}

TEST_F(PhysicsTest, RobotCommandMovesRobot) {
    SimWorld world(cfg);
    // Keep clear of the ball at the origin so the chassis is not resting on it
    world.setBallPose(3.0, 3.0);
    world.setRobotPose(0, Team::Blue, 0, 0, 0);
    // Allow the robot to settle onto the ground
    for (int i = 0; i < 60; i++) world.step();
    auto s0 = world.robotState(0, Team::Blue);
    RobotCommand cmd;
    cmd.id = 0;
    cmd.team = Team::Blue;
    cmd.vel_tangent = 1.5;
    cmd.vel_normal = 0;
    cmd.vel_angular = 0;
    for (int i = 0; i < 180; i++) {
        world.applyCommand(cmd);
        world.step();
    }
    auto s1 = world.robotState(0, Team::Blue);
    double dist = std::hypot(s1.x - s0.x, s1.y - s0.y);
    EXPECT_GT(dist, 0.3) << "robot should have moved under forward velocity"
                          << " (start=" << s0.x << "," << s0.y
                          << " end=" << s1.x << "," << s1.y << ")";
}

TEST_F(PhysicsTest, VisionReportsAllRobots) {
    SimWorld world(cfg);
    auto v = world.captureVision();
    EXPECT_EQ(static_cast<int>(v.robots_blue.size()), cfg.robots_count);
    EXPECT_EQ(static_cast<int>(v.robots_yellow.size()), cfg.robots_count);
}

TEST_F(PhysicsTest, SetRobotPoseWorks) {
    SimWorld world(cfg);
    world.setRobotPose(0, Team::Blue, 1.5, -2.0, 90);
    auto s = world.robotState(0, Team::Blue);
    EXPECT_NEAR(s.x, 1.5, 0.05);
    EXPECT_NEAR(s.y, -2.0, 0.05);
}
