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
}

TEST(Config, LoadDivisionB) {
    // write temp yaml
    auto path = cfgPath("config/default.yaml");
    auto c = SimConfig::loadFromFile(path);
    // manually load B field via full file with division B
    std::string tmp = "/tmp/grsim_divb.yaml";
    {
        std::ofstream f(tmp);
        f << "division: B\nrobots_count: 2\n"
          << "field:\n  B:\n    length: 9.0\n    width: 6.0\n"
          << "    line_width: 0.01\n    radius: 0.5\n    free_kick: 0.7\n"
          << "    penalty_width: 2.0\n    penalty_depth: 1.0\n    penalty_point: 6.0\n"
          << "    margin_touch_line: 0.3\n    margin_goal_line: 0.3\n"
          << "    goal_substitution_area_width: 0.0\n    referee_margin: 0.0\n"
          << "    wall_thickness: 0.05\n    goal_thickness: 0.02\n"
          << "    goal_depth: 0.18\n    goal_width: 1.0\n    goal_height: 0.16\n"
          << "teams:\n  blue: parsian\n  yellow: parsian\n";
    }
    // ensure robot path resolves - run from project root style
    // copy path resolution by setting cwd-relative; loadRobot may fail
    // so we chdir is not available; loadFromFile uses base of yaml dir
    // robot yaml is in config/robots relative to config/default - for /tmp it fails
    // Fix: include full path by writing robot inline is not supported;
    // instead just check field dimensions after load catching robot warning
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
