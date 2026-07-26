#include "grsim/config.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>

namespace grsim {

namespace {

FieldGeometry loadFieldNode(const YAML::Node& n) {
    FieldGeometry f;
    if (!n) return f;
    f.line_width = n["line_width"].as<double>(f.line_width);
    f.length = n["length"].as<double>(f.length);
    f.width = n["width"].as<double>(f.width);
    f.radius = n["radius"].as<double>(f.radius);
    f.free_kick = n["free_kick"].as<double>(f.free_kick);
    f.penalty_width = n["penalty_width"].as<double>(f.penalty_width);
    f.penalty_depth = n["penalty_depth"].as<double>(f.penalty_depth);
    f.penalty_point = n["penalty_point"].as<double>(f.penalty_point);
    f.margin_touch_line = n["margin_touch_line"].as<double>(f.margin_touch_line);
    f.margin_goal_line = n["margin_goal_line"].as<double>(f.margin_goal_line);
    f.goal_substitution_area_width = n["goal_substitution_area_width"].as<double>(f.goal_substitution_area_width);
    f.referee_margin = n["referee_margin"].as<double>(f.referee_margin);
    f.wall_thickness = n["wall_thickness"].as<double>(f.wall_thickness);
    f.goal_thickness = n["goal_thickness"].as<double>(f.goal_thickness);
    f.goal_depth = n["goal_depth"].as<double>(f.goal_depth);
    f.goal_width = n["goal_width"].as<double>(f.goal_width);
    f.goal_height = n["goal_height"].as<double>(f.goal_height);
    return f;
}

RunMode parseMode(const std::string& s) {
    std::string v = s;
    std::transform(v.begin(), v.end(), v.begin(), ::tolower);
    if (v == "async") return RunMode::Async;
    return RunMode::Sync;
}

BehaviorType parseBehavior(const std::string& s) {
    std::string v = s;
    std::transform(v.begin(), v.end(), v.begin(), ::tolower);
    if (v == "square") return BehaviorType::Square;
    if (v == "idle") return BehaviorType::Idle;
    return BehaviorType::Circle;
}

std::string resolveRobotPath(const std::string& name, const std::string& base_dir) {
    // try relative paths commonly used
    std::vector<std::string> candidates = {
        base_dir + "/robots/" + name + ".yaml",
        base_dir + "/" + name + ".yaml",
        "config/robots/" + name + ".yaml",
        "../config/robots/" + name + ".yaml",
        "config/" + name + ".yaml"
    };
    for (const auto& c : candidates) {
        std::ifstream f(c);
        if (f.good()) return c;
    }
    return candidates.back();
}

std::string parentDir(const std::string& path) {
    auto pos = path.find_last_of("/\\");
    if (pos == std::string::npos) return ".";
    return path.substr(0, pos);
}

}  // namespace

RobotSettings SimConfig::loadRobotSettings(const std::string& path) {
    YAML::Node root = YAML::LoadFile(path);
    RobotSettings s;
    auto g = root["geometry"];
    if (g) {
        s.robot_center_from_kicker = g["center_from_kicker"].as<double>(s.robot_center_from_kicker);
        s.robot_radius = g["radius"].as<double>(s.robot_radius);
        s.robot_height = g["height"].as<double>(s.robot_height);
        s.bottom_height = g["bottom_height"].as<double>(s.bottom_height);
        s.kicker_z = g["kicker_z"].as<double>(s.kicker_z);
        s.kicker_thickness = g["kicker_thickness"].as<double>(s.kicker_thickness);
        s.kicker_width = g["kicker_width"].as<double>(s.kicker_width);
        s.kicker_height = g["kicker_height"].as<double>(s.kicker_height);
        s.wheel_radius = g["wheel_radius"].as<double>(s.wheel_radius);
        s.wheel_thickness = g["wheel_thickness"].as<double>(s.wheel_thickness);
        s.wheel1_angle = g["wheel1_angle"].as<double>(s.wheel1_angle);
        s.wheel2_angle = g["wheel2_angle"].as<double>(s.wheel2_angle);
        s.wheel3_angle = g["wheel3_angle"].as<double>(s.wheel3_angle);
        s.wheel4_angle = g["wheel4_angle"].as<double>(s.wheel4_angle);
    }
    auto p = root["physics"];
    if (p) {
        s.body_mass = p["body_mass"].as<double>(s.body_mass);
        s.wheel_mass = p["wheel_mass"].as<double>(s.wheel_mass);
        s.kicker_mass = p["kicker_mass"].as<double>(s.kicker_mass);
        s.kicker_damp_factor = p["kicker_damp_factor"].as<double>(s.kicker_damp_factor);
        s.roller_torque_factor = p["roller_torque_factor"].as<double>(s.roller_torque_factor);
        s.roller_perpendicular_torque_factor = p["roller_perpendicular_torque_factor"].as<double>(s.roller_perpendicular_torque_factor);
        s.kicker_friction = p["kicker_friction"].as<double>(s.kicker_friction);
        s.wheel_tangent_friction = p["wheel_tangent_friction"].as<double>(s.wheel_tangent_friction);
        s.wheel_perpendicular_friction = p["wheel_perpendicular_friction"].as<double>(s.wheel_perpendicular_friction);
        s.wheel_motor_fmax = p["wheel_motor_fmax"].as<double>(s.wheel_motor_fmax);
        s.max_linear_kick_speed = p["max_linear_kick_speed"].as<double>(s.max_linear_kick_speed);
        s.max_chip_kick_speed = p["max_chip_kick_speed"].as<double>(s.max_chip_kick_speed);
        s.acc_speedup_absolute_max = p["acc_speedup_absolute_max"].as<double>(s.acc_speedup_absolute_max);
        s.acc_speedup_angular_max = p["acc_speedup_angular_max"].as<double>(s.acc_speedup_angular_max);
        s.acc_brake_absolute_max = p["acc_brake_absolute_max"].as<double>(s.acc_brake_absolute_max);
        s.acc_brake_angular_max = p["acc_brake_angular_max"].as<double>(s.acc_brake_angular_max);
        s.vel_absolute_max = p["vel_absolute_max"].as<double>(s.vel_absolute_max);
        s.vel_angular_max = p["vel_angular_max"].as<double>(s.vel_angular_max);
    }
    return s;
}

SimConfig SimConfig::defaults() {
    SimConfig c;
    c.applyDivision();
    c.robot_settings = c.blue_robot;
    return c;
}

void SimConfig::applyDivision() {
    // field already set by loader; defaults are Division A
    if (division == "B" || division == "b") {
        // if field still default A sizes, overwrite with B defaults
        // explicit field load handles this; here ensure B defaults if needed
    }
}

SimConfig SimConfig::loadFromFile(const std::string& path) {
    YAML::Node root = YAML::LoadFile(path);
    SimConfig c = defaults();
    std::string base = parentDir(path);

    c.division = root["division"].as<std::string>(c.division);
    c.robots_count = root["robots_count"].as<int>(c.robots_count);
    if (c.robots_count < 1) c.robots_count = 1;
    if (c.robots_count > kMaxRobotCount) c.robots_count = kMaxRobotCount;

    if (root["field"]) {
        std::string div_key = (c.division == "B" || c.division == "b") ? "B" : "A";
        if (root["field"][div_key]) {
            c.field = loadFieldNode(root["field"][div_key]);
        } else if (root["field"]["length"]) {
            c.field = loadFieldNode(root["field"]);
        } else {
            // try A then B
            if (root["field"]["A"]) c.field = loadFieldNode(root["field"]["A"]);
        }
    }

    if (root["ball"]) {
        auto b = root["ball"];
        c.ball.radius = b["radius"].as<double>(c.ball.radius);
        c.ball.mass = b["mass"].as<double>(c.ball.mass);
        c.ball.friction = b["friction"].as<double>(c.ball.friction);
        c.ball.slip = b["slip"].as<double>(c.ball.slip);
        c.ball.bounce = b["bounce"].as<double>(c.ball.bounce);
        c.ball.bounce_vel = b["bounce_vel"].as<double>(c.ball.bounce_vel);
        c.ball.linear_damp = b["linear_damp"].as<double>(c.ball.linear_damp);
        c.ball.angular_damp = b["angular_damp"].as<double>(c.ball.angular_damp);
    }

    if (root["physics"]) {
        auto p = root["physics"];
        c.physics.desired_fps = p["desired_fps"].as<double>(c.physics.desired_fps);
        c.physics.delta_time = p["delta_time"].as<double>(c.physics.delta_time);
        c.physics.gravity = p["gravity"].as<double>(c.physics.gravity);
        c.physics.reset_turnover = p["reset_turnover"].as<bool>(c.physics.reset_turnover);
    }

    if (root["teams"]) {
        c.blue_team_name = root["teams"]["blue"].as<std::string>(c.blue_team_name);
        c.yellow_team_name = root["teams"]["yellow"].as<std::string>(c.yellow_team_name);
    }

    try {
        c.blue_robot = loadRobotSettings(resolveRobotPath(c.blue_team_name, base));
    } catch (const std::exception& e) {
        std::cerr << "Warning: failed to load blue robot settings: " << e.what() << "\n";
    }
    try {
        c.yellow_robot = loadRobotSettings(resolveRobotPath(c.yellow_team_name, base));
    } catch (const std::exception& e) {
        std::cerr << "Warning: failed to load yellow robot settings: " << e.what() << "\n";
    }
    c.robot_settings = c.blue_robot;

    if (root["client"]) {
        auto cl = root["client"];
        c.client.control_period_ms = cl["control_period_ms"].as<int>(c.client.control_period_ms);
        if (cl["mode"]) c.client.mode = parseMode(cl["mode"].as<std::string>());
        if (cl["behavior"]) c.client.behavior = parseBehavior(cl["behavior"].as<std::string>());
        c.client.speed = cl["speed"].as<double>(c.client.speed);
        c.client.circle_radius = cl["circle_radius"].as<double>(c.client.circle_radius);
        c.client.square_size = cl["square_size"].as<double>(c.client.square_size);
        c.client.active_team = cl["active_team"].as<std::string>(c.client.active_team);
        if (cl["active_robots"] && cl["active_robots"].IsScalar()) {
            c.client.active_robots = cl["active_robots"].as<std::string>("all");
        } else {
            c.client.active_robots = "all";
        }
    }

    if (root["logging"]) {
        auto lg = root["logging"];
        c.logging.enabled = lg["enabled"].as<bool>(c.logging.enabled);
        c.logging.directory = lg["directory"].as<std::string>(c.logging.directory);
        c.logging.prefix = lg["prefix"].as<std::string>(c.logging.prefix);
        c.logging.log_vision = lg["log_vision"].as<bool>(c.logging.log_vision);
        c.logging.log_commands = lg["log_commands"].as<bool>(c.logging.log_commands);
    }

    c.formation = root["formation"].as<std::string>(c.formation);
    return c;
}

}  // namespace grsim
