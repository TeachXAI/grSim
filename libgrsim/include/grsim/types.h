#pragma once

#include "grsim/constants.h"
#include <vector>
#include <string>
#include <cstdint>

namespace grsim {

struct Vec2 {
    double x = 0;
    double y = 0;
};

struct RobotCommand {
    int id = 0;
    Team team = Team::Blue;
    double vel_tangent = 0;   // local forward
    double vel_normal = 0;    // local left
    double vel_angular = 0;   // rad/s
    double kick_speed_x = 0;
    double kick_speed_z = 0;
    bool spinner = false;
    bool wheels_speed = false;
    double wheel[4] = {0, 0, 0, 0};
};

struct RobotState {
    int id = 0;
    Team team = Team::Blue;
    double x = 0;
    double y = 0;
    double z = 0;
    double orientation = 0;  // radians
    double vx = 0;
    double vy = 0;
    double omega = 0;
    bool on = true;
    bool infrared = false;
    KickStatus kick = KickStatus::NoKick;
};

struct BallState {
    double x = 0;
    double y = 0;
    double z = 0;
    double vx = 0;
    double vy = 0;
    double vz = 0;
};

struct VisionFrame {
    uint64_t frame_number = 0;
    double t_capture = 0;
    BallState ball;
    std::vector<RobotState> robots_blue;
    std::vector<RobotState> robots_yellow;
};

struct CommandLogEntry {
    double t = 0;
    uint64_t frame = 0;
    RobotCommand command;
};

struct FieldGeometry {
    double length = 12.0;
    double width = 9.0;
    double line_width = 0.01;
    double radius = 0.5;
    double free_kick = 0.7;
    double penalty_width = 3.6;
    double penalty_depth = 1.8;
    double penalty_point = 8.0;
    double margin_touch_line = 0.3;
    double margin_goal_line = 0.6;
    double goal_substitution_area_width = 0.3;
    double referee_margin = 0.0;
    double wall_thickness = 0.05;
    double goal_thickness = 0.02;
    double goal_depth = 0.18;
    double goal_width = 1.8;
    double goal_height = 0.16;
};

}  // namespace grsim
