#include "grsim/world.h"
#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace grsim {

SimWorld* SimWorld::active_ = nullptr;

dReal SimWorld::fric(dReal f) {
    if (f == -1) return dInfinity;
    return f;
}

bool wheelCallBack(dGeomID o1, dGeomID o2, PSurface* s, int /*robots_count*/) {
    SimWorld* w = SimWorld::active_;
    if (!w) return false;
    const dReal* r;
    if ((o1 == s->id1) && (o2 == s->id2)) {
        r = dBodyGetRotation(dGeomGetBody(o1));
    } else if ((o1 == s->id2) && (o2 == s->id1)) {
        r = dBodyGetRotation(dGeomGetBody(o2));
    } else {
        return false;
    }
    s->surface.mode = dContactFDir1 | dContactMu2 | dContactApprox1 | dContactSoftCFM;
    s->surface.mu = SimWorld::fric(w->cfg_.robot_settings.wheel_perpendicular_friction);
    s->surface.mu2 = SimWorld::fric(w->cfg_.robot_settings.wheel_tangent_friction);
    s->surface.soft_cfm = 0.002;
    dVector3 v = {0, 0, 1, 1};
    dVector3 axis;
    dMultiply0(axis, r, v, 4, 3, 1);
    dReal l = sqrt(axis[0] * axis[0] + axis[1] * axis[1]);
    if (l < 1e-9) return false;
    s->fdir1[0] = axis[0] / l;
    s->fdir1[1] = axis[1] / l;
    s->fdir1[2] = 0;
    s->fdir1[3] = 0;
    s->usefdir1 = true;
    return true;
}

bool ballCallBack(dGeomID /*o1*/, dGeomID /*o2*/, PSurface* s, int /*robots_count*/) {
    SimWorld* w = SimWorld::active_;
    if (!w || w->ball_->tag == -1) return true;
    dReal x, y, z;
    w->robots_[w->ball_->tag]->chassis->getBodyDirection(x, y, z);
    s->fdir1[0] = x;
    s->fdir1[1] = y;
    s->fdir1[2] = 0;
    s->fdir1[3] = 0;
    s->usefdir1 = true;
    s->surface.mode = dContactMu2 | dContactFDir1 | dContactSoftCFM;
    s->surface.mu = w->cfg_.BallFriction();
    s->surface.mu2 = 0.5;
    s->surface.soft_cfm = 0.002;
    return true;
}

void RobotsFormation::setAll(const dReal* xx, const dReal* yy) {
    for (int i = 0; i < kMaxRobotCount; i++) {
        x[i] = xx[i];
        y[i] = yy[i];
    }
}

RobotsFormation::RobotsFormation(FormationType type, const SimConfig& cfg) {
    switch (type) {
        case FormationType::Outside: {
            double yv = -(cfg.Field_Width() / 2 + cfg.Field_Margin_Touch_Line() / 2);
            dReal teamPosX[kMaxRobotCount] = {0.40, 0.80, 1.20, 1.60, 2.00, 2.40,
                                              2.80, 3.20, 3.60, 4.00, 4.40, 4.80,
                                              0.40, 0.80, 1.20, 1.60};
            dReal teamPosY[kMaxRobotCount];
            for (int i = 0; i < kMaxRobotCount; i++) teamPosY[i] = yv;
            setAll(teamPosX, teamPosY);
            break;
        }
        case FormationType::Inside1: {
            dReal teamPosX[kMaxRobotCount] = {1.50, 1.50, 1.50, 0.55, 2.50, 3.60,
                                              3.20, 3.20, 3.20, 3.20, 3.20, 3.20,
                                              0.40, 0.80, 1.20, 1.60};
            dReal teamPosY[kMaxRobotCount] = {1.12, 0.0, -1.12, 0.00, 0.00, 0.00,
                                              0.75, -0.75, 1.50, -1.50, 2.25, -2.25,
                                              -3.50, -3.50, -3.50, -3.50};
            setAll(teamPosX, teamPosY);
            break;
        }
        case FormationType::Inside2: {
            dReal teamPosX[kMaxRobotCount] = {4.20, 3.40, 3.40, 0.70, 0.70, 0.70,
                                              2.00, 2.00, 2.00, 2.00, 2.00, 2.00,
                                              0.40, 0.80, 1.20, 1.60};
            dReal teamPosY[kMaxRobotCount] = {0.00, -0.20, 0.20, 0.00, 2.25, -2.25,
                                              0.75, -0.75, 1.50, -1.50, 2.25, -2.25,
                                              -3.50, -3.50, -3.50, -3.50};
            setAll(teamPosX, teamPosY);
            break;
        }
        case FormationType::OutsideField: {
            double yv = -(cfg.Field_Width() / 2 + cfg.Field_Margin_Touch_Line()
                          + cfg.Field_Referee_Margin() + 0.5);
            dReal teamPosX[kMaxRobotCount] = {0.40, 0.80, 1.20, 1.60, 2.00, 2.40,
                                              2.80, 3.20, 3.60, 4.00, 4.40, 4.80,
                                              0.40, 0.80, 1.20, 1.60};
            dReal teamPosY[kMaxRobotCount];
            for (int i = 0; i < kMaxRobotCount; i++) teamPosY[i] = yv;
            setAll(teamPosX, teamPosY);
            break;
        }
    }
}

void RobotsFormation::resetRobots(Robot** r, int team, int robots_count) {
    dReal dir = (team == 1) ? 1 : -1;
    for (int k = 0; k < robots_count; k++) {
        r[k + team * robots_count]->setXY(x[k] * dir, y[k]);
        r[k + team * robots_count]->resetRobot();
    }
}

SimWorld::SimWorld(const SimConfig& config) : cfg_(config) {
    active_ = this;
    for (auto& r : robots_) r = nullptr;
    for (auto& w : walls_) w = nullptr;

    p_ = std::make_unique<PWorld>(cfg_.DeltaTime(), cfg_.Gravity(), cfg_.Robots_Count());
    ball_ = new PBall(0, 0, 0.5, cfg_.BallRadius(), cfg_.BallMass(), 1, 0.7, 0);
    ground_ = new PGround(cfg_.Field_Rad(), cfg_.Field_Length(), cfg_.Field_Width(),
                          cfg_.Field_Penalty_Depth(), cfg_.Field_Penalty_Width(),
                          cfg_.Field_Penalty_Point(), cfg_.Field_Line_Width());

    buildField();

    FormationType ft = FormationType::Outside;
    if (cfg_.formation == "inside_1") ft = FormationType::Inside1;
    else if (cfg_.formation == "inside_2") ft = FormationType::Inside2;
    else if (cfg_.formation == "outside_field") ft = FormationType::OutsideField;

    RobotsFormation form1(ft, cfg_);
    RobotsFormation form2(ft, cfg_);
    buildRobots(form1, form2);

    p_->initAllObjects();
    setupSurfaces();

    // Place ball on ground
    ball_->setBodyPosition(0, 0, cfg_.BallRadius() * 1.2);
    dBodySetLinearVel(ball_->body, 0, 0, 0);
    dBodySetAngularVel(ball_->body, 0, 0, 0);

    last_dt_ = cfg_.DeltaTime();
}

SimWorld::~SimWorld() {
    // Destroy ODE world first while PObject wrappers are still alive so
    // PWorld can null body/geom pointers safely, then free C++ wrappers.
    p_.reset();
    for (int i = 0; i < robot_capacity_; i++) {
        delete robots_[i];
        robots_[i] = nullptr;
    }
    for (auto& wall : walls_) {
        delete wall;
        wall = nullptr;
    }
    delete ball_;
    ball_ = nullptr;
    delete ground_;
    ground_ = nullptr;
    if (active_ == this) active_ = nullptr;
}

void SimWorld::buildField() {
    const double thick = cfg_.Wall_Thickness();
    const double increment_x = cfg_.Field_Margin_Goal_Line() + cfg_.Field_Referee_Margin() + thick / 2;
    const double increment_y = cfg_.Field_Margin_Touch_Line() + cfg_.Field_Referee_Margin() + thick / 2;
    const double pos_x = cfg_.Field_Length() / 2.0 + increment_x;
    const double pos_y = cfg_.Field_Width() / 2.0 + increment_y;
    const double siz_x = 2.0 * pos_x;
    const double siz_y = 2.0 * pos_y;
    const double siz_z = 0.4;
    const double tone = 1.0;

    walls_[0] = new PFixedBox(thick / 2, pos_y, 0, siz_x, thick, siz_z, tone, tone, tone);
    walls_[1] = new PFixedBox(-thick / 2, -pos_y, 0, siz_x, thick, siz_z, tone, tone, tone);
    walls_[2] = new PFixedBox(pos_x, -thick / 2, 0, thick, siz_y, siz_z, tone, tone, tone);
    walls_[3] = new PFixedBox(-pos_x, thick / 2, 0, thick, siz_y, siz_z, tone, tone, tone);

    const double gthick = cfg_.Goal_Thickness();
    const double gpos_x = (cfg_.Field_Length() + gthick) / 2.0 + cfg_.Goal_Depth();
    const double gpos_y = (cfg_.Goal_Width() + gthick) / 2.0;
    const double gpos_z = cfg_.Goal_Height() / 2.0;
    const double gsiz_x = cfg_.Field_Margin_Goal_Line() + cfg_.Field_Referee_Margin();
    const double gsiz_y = cfg_.Goal_Width();
    const double gsiz_z = cfg_.Goal_Height();
    const double gpos2_x = (cfg_.Field_Length() + gsiz_x) / 2.0;

    walls_[4] = new PFixedBox(gpos_x, 0.0, gpos_z, gthick, gsiz_y, gsiz_z, tone, tone, tone);
    walls_[5] = new PFixedBox(gpos2_x, -gpos_y, gpos_z, gsiz_x, gthick, gsiz_z, tone, tone, tone);
    walls_[6] = new PFixedBox(gpos2_x, gpos_y, gpos_z, gsiz_x, gthick, gsiz_z, tone, tone, tone);
    walls_[7] = new PFixedBox(-gpos_x, 0.0, gpos_z, gthick, gsiz_y, gsiz_z, tone, tone, tone);
    walls_[8] = new PFixedBox(-gpos2_x, -gpos_y, gpos_z, gsiz_x, gthick, gsiz_z, tone, tone, tone);
    walls_[9] = new PFixedBox(-gpos2_x, gpos_y, gpos_z, gsiz_x, gthick, gsiz_z, tone, tone, tone);

    p_->addObject(ground_);
    p_->addObject(ball_);
    for (auto& wall : walls_) p_->addObject(wall);
}

void SimWorld::buildRobots(const RobotsFormation& form1, const RobotsFormation& form2) {
    robot_capacity_ = cfg_.Robots_Count() * 2;
    cfg_.robot_settings = cfg_.blue_robot;
    for (int k = 0; k < cfg_.Robots_Count(); k++) {
        robots_[k] = new Robot(p_.get(), ball_, &cfg_,
                               -form1.x[k], form1.y[k], robotStartZ(&cfg_),
                               0.8, 0.8, 0.8, k + 1, 1);
    }
    cfg_.robot_settings = cfg_.yellow_robot;
    for (int k = 0; k < cfg_.Robots_Count(); k++) {
        robots_[k + cfg_.Robots_Count()] = new Robot(p_.get(), ball_, &cfg_,
            form2.x[k], form2.y[k], robotStartZ(&cfg_),
            0.8, 0.8, 0.8, k + cfg_.Robots_Count() + 1, -1);
    }
    // Use blue as default active settings for friction callbacks
    cfg_.robot_settings = cfg_.blue_robot;
}

void SimWorld::setupSurfaces() {
    PSurface ballwithwall;
    ballwithwall.surface.mode = dContactBounce | dContactApprox1;
    ballwithwall.surface.mu = 1;
    ballwithwall.surface.bounce = cfg_.BallBounce();
    ballwithwall.surface.bounce_vel = cfg_.BallBounceVel();
    ballwithwall.surface.slip1 = 0;

    PSurface* ball_ground = p_->createSurface(ball_, ground_);
    ball_ground->surface = ballwithwall.surface;
    ball_ground->callback = ballCallBack;

    PSurface ballwithkicker;
    ballwithkicker.surface.mode = dContactApprox1;
    ballwithkicker.surface.mu = fric(cfg_.robot_settings.kicker_friction);
    ballwithkicker.surface.slip1 = 5;

    for (auto& wall : walls_) p_->createSurface(ball_, wall)->surface = ballwithwall.surface;

    PSurface wheelswithground;
    for (int k = 0; k < robot_capacity_; k++) {
        p_->createSurface(robots_[k]->chassis, ground_);
        for (auto& wall : walls_) p_->createSurface(robots_[k]->chassis, wall);
        p_->createSurface(robots_[k]->dummy, ball_);
        p_->createSurface(robots_[k]->kicker->box, ball_)->surface = ballwithkicker.surface;
        for (auto& wheel : robots_[k]->wheels) {
            p_->createSurface(wheel->cyl, ball_);
            PSurface* w_g = p_->createSurface(wheel->cyl, ground_);
            w_g->surface = wheelswithground.surface;
            w_g->usefdir1 = true;
            w_g->callback = wheelCallBack;
        }
        for (int j = k + 1; j < robot_capacity_; j++) {
            p_->createSurface(robots_[k]->dummy, robots_[j]->dummy);
            p_->createSurface(robots_[k]->chassis, robots_[j]->kicker->box);
        }
    }
}

int SimWorld::robotIndex(int robot, int team) const {
    if (robot < 0 || robot >= cfg_.Robots_Count()) return -1;
    return robot + team * cfg_.Robots_Count();
}

Robot* SimWorld::robot(int id, Team team) {
    int idx = robotIndex(id, static_cast<int>(team));
    if (idx < 0) return nullptr;
    return robots_[idx];
}

const Robot* SimWorld::robot(int id, Team team) const {
    int idx = robotIndex(id, static_cast<int>(team));
    if (idx < 0) return nullptr;
    return robots_[idx];
}

void SimWorld::step(double dt) {
    active_ = this;
    drainCommandQueue();

    if (dt < 0) dt = cfg_.DeltaTime();
    last_dt_ = dt;

    const int ballCollisionTry = 5;
    for (int kk = 0; kk < ballCollisionTry; kk++) {
        const dReal* ballvel = dBodyGetLinearVel(ball_->body);
        dReal ballspeed = sqrt(ballvel[0] * ballvel[0] + ballvel[1] * ballvel[1] + ballvel[2] * ballvel[2]);
        if (ballspeed > 0.01) {
            dReal fk = cfg_.BallFriction() * cfg_.BallMass() * cfg_.Gravity();
            dReal ballfx = -fk * ballvel[0] / ballspeed;
            dReal ballfy = -fk * ballvel[1] / ballspeed;
            dReal ballfz = -fk * ballvel[2] / ballspeed;
            dBodyAddTorque(ball_->body, -ballfy * cfg_.BallRadius(), ballfx * cfg_.BallRadius(), 0);
            dBodyAddForce(ball_->body, ballfx, ballfy, ballfz);
        } else {
            dBodySetAngularVel(ball_->body, 0, 0, 0);
            dBodySetLinearVel(ball_->body, 0, 0, 0);
        }
        p_->step(dt / ballCollisionTry);
    }

    sim_time_ += last_dt_;
    ball_->tag = -1;
    for (int k = 0; k < robot_capacity_; k++) {
        if (cfg_.ResetTurnOver()) {
            dReal kk;
            robots_[k]->getDir(kk);
            if (kk < 0.9) robots_[k]->resetRobot();
        }
        robots_[k]->step();
    }
    frame_num_++;
}

void SimWorld::applyCommand(const RobotCommand& cmd) {
    int id = robotIndex(cmd.id, static_cast<int>(cmd.team));
    if (id < 0 || id >= robot_capacity_) return;
    Robot* r = robots_[id];
    if (!r || !r->on) return;

    // Use team-specific robot settings for limits
    cfg_.robot_settings = (cmd.team == Team::Yellow) ? cfg_.yellow_robot : cfg_.blue_robot;

    if (cmd.wheels_speed) {
        for (int i = 0; i < 4; i++) r->setSpeed(i, cmd.wheel[i]);
    } else {
        r->setSpeed(cmd.vel_tangent, cmd.vel_normal, cmd.vel_angular);
    }
    if (cmd.kick_speed_x > 0.0001 || cmd.kick_speed_z > 0.0001) {
        r->kicker->kick(cmd.kick_speed_x, cmd.kick_speed_z);
    }
    r->kicker->setRoller(cmd.spinner ? 1 : 0);
}

void SimWorld::applyCommands(const std::vector<RobotCommand>& cmds) {
    for (const auto& c : cmds) applyCommand(c);
}

void SimWorld::enqueueCommands(const std::vector<RobotCommand>& cmds) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    pending_cmds_.insert(pending_cmds_.end(), cmds.begin(), cmds.end());
}

void SimWorld::drainCommandQueue() {
    std::vector<RobotCommand> local;
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        local.swap(pending_cmds_);
    }
    applyCommands(local);
}

BallState SimWorld::ballState() const {
    BallState b;
    dReal x, y, z;
    ball_->getBodyPosition(x, y, z);
    b.x = x; b.y = y; b.z = z;
    const dReal* v = dBodyGetLinearVel(ball_->body);
    b.vx = v[0]; b.vy = v[1]; b.vz = v[2];
    return b;
}

RobotState SimWorld::robotState(int id, Team team) const {
    RobotState s;
    s.id = id;
    s.team = team;
    const Robot* r = robot(id, team);
    if (!r) return s;
    dReal x, y;
    r->getXY(x, y);
    s.x = x; s.y = y;
    dReal xx, yy, zz;
    r->chassis->getBodyPosition(xx, yy, zz);
    s.z = zz;
    s.orientation = r->getDir() * M_PI / 180.0;
    const dReal* v = dBodyGetLinearVel(r->chassis->body);
    s.vx = v[0]; s.vy = v[1];
    const dReal* w = dBodyGetAngularVel(r->chassis->body);
    s.omega = w[2];
    s.on = r->on;
    s.infrared = r->kicker->isTouchingBall();
    s.kick = r->kicker->isKicking();
    return s;
}

std::vector<RobotState> SimWorld::allRobots() const {
    std::vector<RobotState> out;
    out.reserve(robot_capacity_);
    for (int t = 0; t < 2; t++) {
        for (int i = 0; i < cfg_.Robots_Count(); i++) {
            out.push_back(robotState(i, static_cast<Team>(t)));
        }
    }
    return out;
}

VisionFrame SimWorld::captureVision() const {
    VisionFrame f;
    f.frame_number = frame_num_;
    f.t_capture = sim_time_;
    f.ball = ballState();
    for (int i = 0; i < cfg_.Robots_Count(); i++) {
        auto rb = robotState(i, Team::Blue);
        if (rb.on) f.robots_blue.push_back(rb);
        auto ry = robotState(i, Team::Yellow);
        if (ry.on) f.robots_yellow.push_back(ry);
    }
    return f;
}

void SimWorld::setRobotPose(int id, Team team, double x, double y, double dir_deg) {
    Robot* r = robot(id, team);
    if (!r) return;
    r->setXY(x, y);
    r->resetRobot();
    r->setDir(dir_deg);
}

void SimWorld::setBallPose(double x, double y, double z, double vx, double vy, double vz) {
    if (z < 0) z = cfg_.BallRadius() * 1.2;
    ball_->setBodyPosition(x, y, z);
    dBodySetLinearVel(ball_->body, vx, vy, vz);
    dBodySetAngularVel(ball_->body, 0, 0, 0);
}

}  // namespace grsim
