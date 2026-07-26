#pragma once

#include "grsim/types.h"
#include "grsim/constants.h"
#include <string>
#include <stdexcept>

namespace grsim {

struct RobotSettings {
    // geometric
    double robot_center_from_kicker = 0.073;
    double robot_radius = 0.09;
    double robot_height = 0.147;
    double bottom_height = 0.02;
    double kicker_z = 0.005;
    double kicker_thickness = 0.005;
    double kicker_width = 0.08;
    double kicker_height = 0.04;
    double wheel_radius = 0.027;
    double wheel_thickness = 0.005;
    double wheel1_angle = 60.0;
    double wheel2_angle = 135.0;
    double wheel3_angle = 225.0;
    double wheel4_angle = 300.0;
    // physical
    double body_mass = 2.0;
    double wheel_mass = 0.2;
    double kicker_mass = 0.02;
    double kicker_damp_factor = 0.2;
    double roller_torque_factor = 0.06;
    double roller_perpendicular_torque_factor = 0.005;
    double kicker_friction = 0.8;
    double wheel_tangent_friction = 0.8;
    double wheel_perpendicular_friction = 0.05;
    double wheel_motor_fmax = 0.2;
    double max_linear_kick_speed = 10.0;
    double max_chip_kick_speed = 10.0;
    double acc_speedup_absolute_max = 4.0;
    double acc_speedup_angular_max = 50.0;
    double acc_brake_absolute_max = 4.0;
    double acc_brake_angular_max = 50.0;
    double vel_absolute_max = 5.0;
    double vel_angular_max = 20.0;
};

struct BallSettings {
    double radius = 0.0215;
    double mass = 0.043;
    double friction = 0.05;
    double slip = 1.0;
    double bounce = 0.5;
    double bounce_vel = 0.1;
    double linear_damp = 0.004;
    double angular_damp = 0.004;
};

struct PhysicsSettings {
    double desired_fps = 60.0;
    double delta_time = 1.0 / 60.0;
    double gravity = 9.81;
    bool reset_turnover = true;
};

struct ClientSettings {
    int control_period_ms = 16;
    RunMode mode = RunMode::Sync;
    BehaviorType behavior = BehaviorType::Circle;
    double speed = 1.0;
    double circle_radius = 1.5;
    double square_size = 2.0;
    std::string active_team = "both";  // blue | yellow | both
    std::string active_robots = "all";
};

struct LoggingSettings {
    bool enabled = true;
    std::string directory = "output/logs";
    std::string prefix = "run";
    bool log_vision = true;
    bool log_commands = true;
};

struct SimConfig {
    std::string division = "A";
    int robots_count = 3;
    FieldGeometry field;
    BallSettings ball;
    PhysicsSettings physics;
    RobotSettings blue_robot;
    RobotSettings yellow_robot;
    ClientSettings client;
    LoggingSettings logging;
    std::string formation = "outside";
    std::string blue_team_name = "parsian";
    std::string yellow_team_name = "parsian";

    // Active robot settings used during construction (mirrors old robotSettings)
    RobotSettings robot_settings;

    // Convenience accessors matching original API style
    int Robots_Count() const { return robots_count; }
    double Field_Length() const { return field.length; }
    double Field_Width() const { return field.width; }
    double Field_Line_Width() const { return field.line_width; }
    double Field_Rad() const { return field.radius; }
    double Field_Free_Kick() const { return field.free_kick; }
    double Field_Penalty_Width() const { return field.penalty_width; }
    double Field_Penalty_Depth() const { return field.penalty_depth; }
    double Field_Penalty_Point() const { return field.penalty_point; }
    double Field_Margin_Touch_Line() const { return field.margin_touch_line; }
    double Field_Margin_Goal_Line() const { return field.margin_goal_line; }
    double Field_Goal_Substitution_Area_Width() const { return field.goal_substitution_area_width; }
    double Field_Referee_Margin() const { return field.referee_margin; }
    double Wall_Thickness() const { return field.wall_thickness; }
    double Goal_Thickness() const { return field.goal_thickness; }
    double Goal_Depth() const { return field.goal_depth; }
    double Goal_Width() const { return field.goal_width; }
    double Goal_Height() const { return field.goal_height; }
    double BallRadius() const { return ball.radius; }
    double BallMass() const { return ball.mass; }
    double BallFriction() const { return ball.friction; }
    double BallSlip() const { return ball.slip; }
    double BallBounce() const { return ball.bounce; }
    double BallBounceVel() const { return ball.bounce_vel; }
    double BallLinearDamp() const { return ball.linear_damp; }
    double BallAngularDamp() const { return ball.angular_damp; }
    double DesiredFPS() const { return physics.desired_fps; }
    double DeltaTime() const { return physics.delta_time; }
    double Gravity() const { return physics.gravity; }
    bool ResetTurnOver() const { return physics.reset_turnover; }

    static SimConfig loadFromFile(const std::string& path);
    static SimConfig defaults();
    static RobotSettings loadRobotSettings(const std::string& path);
    void applyDivision();
};

}  // namespace grsim
