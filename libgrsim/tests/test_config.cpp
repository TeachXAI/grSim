#include <gtest/gtest.h>
#include "grsim/config.h"
#include <fstream>

using namespace grsim;

static std::string cfgPath(const std::string& rel) {
#ifdef GRSIM_SOURCE_DIR
    return std::string(GRSIM_SOURCE_DIR) + "/" + rel;
#else
    return rel;
#endif
}

TEST(Config, DefaultsAreValid) {
    auto c = SimConfig::defaults();
    EXPECT_GT(c.Robots_Count(), 0);
    EXPECT_GT(c.Field_Length(), 0);
    EXPECT_GT(c.Field_Width(), 0);
    EXPECT_GT(c.BallRadius(), 0);
    EXPECT_GT(c.DeltaTime(), 0);
    EXPECT_NEAR(c.Gravity(), 9.81, 1e-6);
}

TEST(Config, LoadDefaultYaml) {
    auto path = cfgPath("config/default.yaml");
    auto c = SimConfig::loadFromFile(path);
    EXPECT_EQ(c.division, "A");
    EXPECT_EQ(c.robots_count, 3);
    EXPECT_NEAR(c.Field_Length(), 12.0, 1e-6);
    EXPECT_NEAR(c.Field_Width(), 9.0, 1e-6);
    EXPECT_NEAR(c.blue_robot.robot_radius, 0.09, 1e-6);
    EXPECT_NEAR(c.blue_robot.wheel_radius, 0.027, 1e-6);
    EXPECT_EQ(c.client.control_period_ms, 16);
    EXPECT_EQ(c.client.mode, RunMode::Sync);
    EXPECT_EQ(c.client.behavior, BehaviorType::Circle);
    // env section
    EXPECT_NEAR(c.env.max_episode_time, 60.0, 1e-6);
    EXPECT_EQ(c.env.reward_type, "zero");
}

TEST(Config, LoadDivisionB) {
    std::string tmp = "/tmp/grsim_divb.yaml";
    {
        std::ofstream f(tmp);
        f << "simulation:\n  division: B\n  robots_count: 2\n"
          << "field:\n  B:\n    length: 9.0\n    width: 6.0\n"
          << "    line_width: 0.01\n    radius: 0.5\n    free_kick: 0.7\n"
          << "    penalty_width: 2.0\n    penalty_depth: 1.0\n    penalty_point: 6.0\n"
          << "    margin_touch_line: 0.3\n    margin_goal_line: 0.3\n"
          << "    goal_substitution_area_width: 0.0\n    referee_margin: 0.0\n"
          << "    wall_thickness: 0.05\n    goal_thickness: 0.02\n"
          << "    goal_depth: 0.18\n    goal_width: 1.0\n    goal_height: 0.16\n"
          << "robots:\n  blue: parsian\n  yellow: parsian\n";
    }
    auto c2 = SimConfig::loadFromFile(tmp);
    EXPECT_NEAR(c2.Field_Length(), 9.0, 1e-6);
    EXPECT_NEAR(c2.Field_Width(), 6.0, 1e-6);
    EXPECT_EQ(c2.robots_count, 2);
}

TEST(Config, LoadRobotYaml) {
    auto path = cfgPath("config/robots/parsian.yaml");
    auto s = SimConfig::loadRobotSettings(path);
    EXPECT_NEAR(s.robot_radius, 0.09, 1e-9);
    EXPECT_NEAR(s.body_mass, 2.0, 1e-9);
    EXPECT_NEAR(s.vel_absolute_max, 5.0, 1e-9);
    EXPECT_NEAR(s.wheel1_angle, 60.0, 1e-9);
}

TEST(Config, HierarchicalControlAndEnv) {
    std::string tmp = "/tmp/grsim_hier.yaml";
    {
        std::ofstream f(tmp);
        f << "simulation:\n  division: A\n  robots_count: 4\n  formation: inside_1\n  seed: 99\n"
          << "control:\n  mode: async\n  behavior: square\n  control_period_ms: 20\n"
          << "  speed: 1.2\n  circle_radius: 2.0\n  square_size: 3.0\n"
          << "env:\n  max_episode_time: 12.5\n  reward_type: ball_progress\n"
          << "  terminate_on_goal: true\n  physics_steps_per_action: 2\n"
          << "logging:\n  enabled: false\n"
          << "teams:\n  blue: parsian\n  yellow: parsian\n";
    }
    auto c = SimConfig::loadFromFile(tmp);
    EXPECT_EQ(c.robots_count, 4);
    EXPECT_EQ(c.formation, "inside_1");
    EXPECT_EQ(c.client.mode, RunMode::Async);
    EXPECT_EQ(c.client.behavior, BehaviorType::Square);
    EXPECT_EQ(c.client.control_period_ms, 20);
    EXPECT_NEAR(c.client.speed, 1.2, 1e-9);
    EXPECT_NEAR(c.env.max_episode_time, 12.5, 1e-9);
    EXPECT_EQ(c.env.reward_type, "ball_progress");
    EXPECT_TRUE(c.env.terminate_on_goal);
    EXPECT_EQ(c.env.physics_steps_per_action, 2);
    EXPECT_TRUE(c.env.seed_enabled);
    EXPECT_EQ(c.env.seed, 99u);
    EXPECT_FALSE(c.logging.enabled);
}

TEST(Config, LegacyClientKeyStillWorks) {
    std::string tmp = "/tmp/grsim_legacy_client.yaml";
    {
        std::ofstream f(tmp);
        f << "division: A\nrobots_count: 2\n"
          << "client:\n  mode: sync\n  behavior: idle\n  control_period_ms: 32\n"
          << "teams:\n  blue: parsian\n  yellow: parsian\n";
    }
    auto c = SimConfig::loadFromFile(tmp);
    EXPECT_EQ(c.client.behavior, BehaviorType::Idle);
    EXPECT_EQ(c.client.control_period_ms, 32);
    EXPECT_EQ(c.robots_count, 2);
}

TEST(Config, InvalidFileThrows) {
    EXPECT_THROW(SimConfig::loadFromFile("/tmp/definitely_missing_grsim_cfg_xyz.yaml"),
                 std::runtime_error);
}

TEST(Config, EmptyFileThrows) {
    std::string tmp = "/tmp/grsim_empty.yaml";
    {
        std::ofstream f(tmp);
        // truly empty
    }
    EXPECT_THROW(SimConfig::loadFromFile(tmp), std::runtime_error);
}
