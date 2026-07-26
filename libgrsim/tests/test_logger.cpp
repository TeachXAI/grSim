#include <gtest/gtest.h>
#include "grsim/logger.h"
#include "grsim/types.h"
#include <fstream>
#include <string>

using namespace grsim;

TEST(Logger, WritesVisionAndCommands) {
    LoggingSettings s;
    s.enabled = true;
    s.directory = "/tmp/grsim_test_logs";
    s.prefix = "logger";
    s.log_vision = true;
    s.log_commands = true;

    SimLogger logger;
    logger.open(s, "logger_unit");
    ASSERT_TRUE(logger.isOpen());

    VisionFrame f;
    f.frame_number = 1;
    f.t_capture = 0.016;
    f.ball = {0.1, 0.2, 0.02, 0, 0, 0};
    RobotState r;
    r.id = 0; r.team = Team::Blue; r.x = 1; r.y = 2; r.orientation = 0.5;
    f.robots_blue.push_back(r);
    logger.logVision(f);

    RobotCommand c;
    c.id = 0; c.team = Team::Blue; c.vel_tangent = 1.0;
    logger.logCommands(0.016, 1, {c});
    logger.close();

    std::ifstream vin(logger.visionPath());
    ASSERT_TRUE(vin.good());
    std::string header;
    std::getline(vin, header);
    EXPECT_NE(header.find("ball_x"), std::string::npos);
    std::string line;
    ASSERT_TRUE(static_cast<bool>(std::getline(vin, line)));
    EXPECT_NE(line.find("blue"), std::string::npos);

    std::ifstream cin(logger.commandPath());
    ASSERT_TRUE(cin.good());
    std::getline(cin, header);
    ASSERT_TRUE(static_cast<bool>(std::getline(cin, line)));
    // iostream may print 1.0 as "1" or "1.0"
    EXPECT_TRUE(line.find("1") != std::string::npos);
    EXPECT_NE(line.find("blue"), std::string::npos);
}
