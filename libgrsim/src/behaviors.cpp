#include "grsim/behaviors.h"
#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace grsim {

namespace {
int key(Team t, int id) { return static_cast<int>(t) * 100 + id; }

bool teamActive(const std::string& active, Team t) {
    if (active == "both") return true;
    if (active == "blue") return t == Team::Blue;
    if (active == "yellow") return t == Team::Yellow;
    return true;
}
}  // namespace

RobotCommand gotoPoint(const RobotState& robot, double tx, double ty,
                       double speed, double kp) {
    RobotCommand cmd;
    cmd.id = robot.id;
    cmd.team = robot.team;

    double dx = tx - robot.x;
    double dy = ty - robot.y;
    double dist = std::hypot(dx, dy);

    // Desired heading toward target (global)
    double desired = std::atan2(dy, dx);
    double err = desired - robot.orientation;
    while (err > M_PI) err -= 2 * M_PI;
    while (err < -M_PI) err += 2 * M_PI;

    // Global velocity toward target, then convert to robot local frame
    double v_scale = std::min(speed, kp * dist);
    double vgx = 0, vgy = 0;
    if (dist > 1e-4) {
        vgx = v_scale * dx / dist;
        vgy = v_scale * dy / dist;
    }
    double c = std::cos(robot.orientation);
    double s = std::sin(robot.orientation);
    // local: forward (tangent) = global · heading, left (normal) = global · left
    cmd.vel_tangent = vgx * c + vgy * s;
    cmd.vel_normal = -vgx * s + vgy * c;
    cmd.vel_angular = 3.0 * err;  // face along path
    return cmd;
}

CircleBehavior::CircleBehavior(const SimConfig& cfg)
    : cfg_(cfg), radius_(cfg.client.circle_radius), speed_(cfg.client.speed) {}

void CircleBehavior::reset() { phase_.clear(); }

bool CircleBehavior::isActive(Team team, int id) const {
    if (!teamActive(cfg_.client.active_team, team)) return false;
    if (cfg_.client.active_robots == "all") return true;
    return true;
}

std::vector<RobotCommand> CircleBehavior::compute(const VisionFrame& vision, double t) {
    std::vector<RobotCommand> cmds;
    auto handle = [&](const RobotState& r) {
        if (!isActive(r.team, r.id)) return;
        int k = key(r.team, r.id);
        if (!phase_.count(k)) {
            // seed phase from current angle so motion is continuous
            phase_[k] = std::atan2(r.y, r.x) + (r.team == Team::Yellow ? M_PI : 0);
            // spread robots by id
            phase_[k] += r.id * (2 * M_PI / std::max(1, cfg_.robots_count));
        }
        double omega = (radius_ > 1e-6) ? (speed_ / radius_) : 0;
        // blue team CCW, yellow CW offset
        double sign = (r.team == Team::Blue) ? 1.0 : -1.0;
        phase_[k] += sign * omega * (cfg_.client.control_period_ms / 1000.0);
        // also advance with absolute time for determinism when reset
        double ang = phase_[k];
        double cx = (r.team == Team::Blue) ? -1.5 : 1.5;
        double cy = 0.0;
        double tx = cx + radius_ * std::cos(ang);
        double ty = cy + radius_ * std::sin(ang);
        cmds.push_back(gotoPoint(r, tx, ty, speed_));
    };
    for (const auto& r : vision.robots_blue) handle(r);
    for (const auto& r : vision.robots_yellow) handle(r);
    (void)t;
    return cmds;
}

SquareBehavior::SquareBehavior(const SimConfig& cfg)
    : cfg_(cfg), size_(cfg.client.square_size), speed_(cfg.client.speed) {}

void SquareBehavior::reset() { progress_.clear(); }

bool SquareBehavior::isActive(Team team, int id) const {
    if (!teamActive(cfg_.client.active_team, team)) return false;
    return true;
}

void SquareBehavior::squarePoint(double perimeter_pos, double half,
                                 double& x, double& y, double& tx, double& ty) {
    double per = 8.0 * half;
    double p = std::fmod(perimeter_pos, per);
    if (p < 0) p += per;
    // sides: bottom, right, top, left  (centered at origin)
    if (p < 2 * half) {
        x = -half + p; y = -half;
        tx = 1; ty = 0;
    } else if (p < 4 * half) {
        x = half; y = -half + (p - 2 * half);
        tx = 0; ty = 1;
    } else if (p < 6 * half) {
        x = half - (p - 4 * half); y = half;
        tx = -1; ty = 0;
    } else {
        x = -half; y = half - (p - 6 * half);
        tx = 0; ty = -1;
    }
}

std::vector<RobotCommand> SquareBehavior::compute(const VisionFrame& vision, double t) {
    std::vector<RobotCommand> cmds;
    double half = size_ / 2.0;
    double dt = cfg_.client.control_period_ms / 1000.0;
    auto handle = [&](const RobotState& r) {
        if (!isActive(r.team, r.id)) return;
        int k = key(r.team, r.id);
        if (!progress_.count(k)) {
            progress_[k] = r.id * (size_ * 2.0);  // stagger
            if (r.team == Team::Yellow) progress_[k] += size_;
        }
        progress_[k] += speed_ * dt;
        double px, py, txx, tyy;
        squarePoint(progress_[k], half, px, py, txx, tyy);
        // offset squares slightly per team
        double ox = (r.team == Team::Blue) ? -0.5 : 0.5;
        cmds.push_back(gotoPoint(r, px + ox, py, speed_));
    };
    for (const auto& r : vision.robots_blue) handle(r);
    for (const auto& r : vision.robots_yellow) handle(r);
    (void)t;
    return cmds;
}

std::vector<RobotCommand> IdleBehavior::compute(const VisionFrame& /*vision*/, double /*t*/) {
    return {};
}

std::unique_ptr<ClientController> createBehavior(const SimConfig& cfg) {
    switch (cfg.client.behavior) {
        case BehaviorType::Square:
            return std::make_unique<SquareBehavior>(cfg);
        case BehaviorType::Idle:
            return std::make_unique<IdleBehavior>();
        case BehaviorType::Circle:
        default:
            return std::make_unique<CircleBehavior>(cfg);
    }
}

}  // namespace grsim
