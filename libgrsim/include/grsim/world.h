#pragma once

#include "grsim/config.h"
#include "grsim/types.h"
#include "grsim/robot.h"
#include "grsim/physics/pworld.h"
#include "grsim/physics/pball.h"
#include "grsim/physics/pground.h"
#include "grsim/physics/pfixedbox.h"
#include <array>
#include <memory>
#include <mutex>
#include <vector>

namespace grsim {

enum class FormationType {
    Outside = 0,
    Inside1 = 1,
    Inside2 = 2,
    OutsideField = 3
};

class RobotsFormation {
public:
    dReal x[kMaxRobotCount]{};
    dReal y[kMaxRobotCount]{};
    explicit RobotsFormation(FormationType type, const SimConfig& cfg);
    void setAll(const dReal* xx, const dReal* yy);
    void resetRobots(Robot** r, int team, int robots_count);
};

class SimWorld {
public:
    explicit SimWorld(const SimConfig& config);
    ~SimWorld();

    // Advance physics by dt seconds (default uses config delta_time)
    void step(double dt = -1);

    // Apply commands in-process (no network)
    void applyCommand(const RobotCommand& cmd);
    void applyCommands(const std::vector<RobotCommand>& cmds);

    // State access
    VisionFrame captureVision() const;
    BallState ballState() const;
    RobotState robotState(int id, Team team) const;
    std::vector<RobotState> allRobots() const;

    void setRobotPose(int id, Team team, double x, double y, double dir_deg);
    void setBallPose(double x, double y, double z = -1,
                     double vx = 0, double vy = 0, double vz = 0);

    static dReal fric(dReal f);
    double simTime() const { return sim_time_; }
    uint64_t frameNumber() const { return frame_num_; }
    const SimConfig& config() const { return cfg_; }
    SimConfig& config() { return cfg_; }

    int robotIndex(int robot, int team) const;
    Robot* robot(int id, Team team);
    const Robot* robot(int id, Team team) const;

    // Thread-safe command queue for async mode
    void enqueueCommands(const std::vector<RobotCommand>& cmds);
    void drainCommandQueue();

private:
    void buildField();
    void buildRobots(const RobotsFormation& form1, const RobotsFormation& form2);
    void setupSurfaces();

    SimConfig cfg_;
    std::unique_ptr<PWorld> p_;
    PBall* ball_ = nullptr;
    PGround* ground_ = nullptr;
    PFixedBox* walls_[kWallCount]{};
    Robot* robots_[kMaxRobotCount * 2]{};
    int robot_capacity_ = 0;

    double sim_time_ = 0;
    double last_dt_ = 0.016;
    uint64_t frame_num_ = 0;

    mutable std::mutex cmd_mutex_;
    std::vector<RobotCommand> pending_cmds_;

    // for wheel friction callback
    static SimWorld* active_;
    friend bool wheelCallBack(dGeomID o1, dGeomID o2, PSurface* s, int robots_count);
    friend bool ballCallBack(dGeomID o1, dGeomID o2, PSurface* s, int robots_count);
};

}  // namespace grsim
