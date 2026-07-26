#pragma once

#include "grsim/client.h"
#include "grsim/config.h"
#include <cmath>
#include <vector>
#include <string>
#include <unordered_map>

namespace grsim {

// Sends all active robots around a circle (center at origin by default)
class CircleBehavior : public ClientController {
public:
    explicit CircleBehavior(const SimConfig& cfg);
    std::vector<RobotCommand> compute(const VisionFrame& vision, double t) override;
    void reset() override;
private:
    SimConfig cfg_;
    double radius_;
    double speed_;
    std::unordered_map<int, double> phase_;  // key = team*100+id
    bool isActive(Team team, int id) const;
};

// Sends robots along a square path
class SquareBehavior : public ClientController {
public:
    explicit SquareBehavior(const SimConfig& cfg);
    std::vector<RobotCommand> compute(const VisionFrame& vision, double t) override;
    void reset() override;
private:
    SimConfig cfg_;
    double size_;
    double speed_;
    std::unordered_map<int, double> progress_;  // distance along perimeter
    bool isActive(Team team, int id) const;
    static void squarePoint(double perimeter_pos, double half, double& x, double& y, double& tx, double& ty);
};

class IdleBehavior : public ClientController {
public:
    std::vector<RobotCommand> compute(const VisionFrame& vision, double t) override;
};

std::unique_ptr<ClientController> createBehavior(const SimConfig& cfg);

// Simple P-controller toward a target position (global frame -> local velocity)
RobotCommand gotoPoint(const RobotState& robot, double tx, double ty,
                       double speed, double kp = 2.0);

}  // namespace grsim
