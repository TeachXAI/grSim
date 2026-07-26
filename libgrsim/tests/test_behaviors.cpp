#include <gtest/gtest.h>
#include "grsim/behaviors.h"
#include "grsim/world.h"

using namespace grsim;

TEST(Behaviors, CircleProducesCommands) {
    SimConfig cfg = SimConfig::defaults();
    cfg.robots_count = 2;
    cfg.client.behavior = BehaviorType::Circle;
    SimWorld world(cfg);
    CircleBehavior beh(cfg);
    auto vision = world.captureVision();
    auto cmds = beh.compute(vision, 0.0);
    EXPECT_FALSE(cmds.empty());
    for (const auto& c : cmds) {
        EXPECT_TRUE(std::isfinite(c.vel_tangent));
        EXPECT_TRUE(std::isfinite(c.vel_normal));
        EXPECT_TRUE(std::isfinite(c.vel_angular));
    }
}

TEST(Behaviors, SquareProducesCommands) {
    SimConfig cfg = SimConfig::defaults();
    cfg.robots_count = 2;
    cfg.client.behavior = BehaviorType::Square;
    SimWorld world(cfg);
    SquareBehavior beh(cfg);
    auto cmds = beh.compute(world.captureVision(), 0.0);
    EXPECT_FALSE(cmds.empty());
}

TEST(Behaviors, IdleProducesNone) {
    IdleBehavior beh;
    VisionFrame f;
    auto cmds = beh.compute(f, 0.0);
    EXPECT_TRUE(cmds.empty());
}

TEST(Behaviors, GotoPointDrivesTowardTarget) {
    RobotState r;
    r.id = 0;
    r.team = Team::Blue;
    r.x = 0; r.y = 0; r.orientation = 0;
    auto cmd = gotoPoint(r, 1.0, 0.0, 1.0);
    EXPECT_GT(cmd.vel_tangent, 0.0);
}

TEST(Behaviors, CreateBehaviorFactory) {
    SimConfig cfg = SimConfig::defaults();
    cfg.client.behavior = BehaviorType::Square;
    auto b = createBehavior(cfg);
    ASSERT_NE(b, nullptr);
}
