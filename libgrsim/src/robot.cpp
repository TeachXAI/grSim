#include "grsim/robot.h"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace grsim {

Robot::Wheel::Wheel(Robot* robot, int _id, dReal ang, dReal ang2) {
    id = _id;
    rob = robot;
    dReal rad = rob->cfg->robot_settings.robot_radius - rob->cfg->robot_settings.wheel_thickness / 2.0;
    ang *= M_PI / 180.0;
    ang2 *= M_PI / 180.0;
    dReal x = rob->m_x;
    dReal y = rob->m_y;
    dReal z = rob->m_z;
    dReal centerx = x + rad * cos(ang2);
    dReal centery = y + rad * sin(ang2);
    dReal centerz = z - rob->cfg->robot_settings.robot_height * 0.5
                      + rob->cfg->robot_settings.wheel_radius
                      - rob->cfg->robot_settings.bottom_height;
    cyl = new PCylinder(centerx, centery, centerz,
                        rob->cfg->robot_settings.wheel_radius,
                        rob->cfg->robot_settings.wheel_thickness,
                        rob->cfg->robot_settings.wheel_mass, 0.9, 0.9, 0.9);
    cyl->setRotation(-sin(ang), cos(ang), 0, M_PI * 0.5);
    cyl->setBodyRotation(-sin(ang), cos(ang), 0, M_PI * 0.5, true);
    cyl->setBodyPosition(centerx - x, centery - y, centerz - z, true);
    cyl->space = rob->space;
    rob->w->addObject(cyl);

    joint = dJointCreateHinge(rob->w->world, 0);
    dJointAttach(joint, rob->chassis->body, cyl->body);
    const dReal* a = dBodyGetPosition(cyl->body);
    dJointSetHingeAxis(joint, cos(ang), sin(ang), 0);
    dJointSetHingeAnchor(joint, a[0], a[1], a[2]);

    motor = dJointCreateAMotor(rob->w->world, 0);
    dJointAttach(motor, rob->chassis->body, cyl->body);
    dJointSetAMotorNumAxes(motor, 1);
    dJointSetAMotorAxis(motor, 0, 1, cos(ang), sin(ang), 0);
    dJointSetAMotorParam(motor, dParamFMax, rob->cfg->robot_settings.wheel_motor_fmax);
    speed = 0;
}

void Robot::Wheel::step() {
    dJointSetAMotorParam(motor, dParamVel, speed);
    dJointSetAMotorParam(motor, dParamFMax, rob->cfg->robot_settings.wheel_motor_fmax);
}

Robot::Kicker::Kicker(Robot* robot) : holdingBall(false) {
    rob = robot;
    dReal x = rob->m_x;
    dReal y = rob->m_y;
    dReal z = rob->m_z;
    dReal centerx = x + (rob->cfg->robot_settings.robot_center_from_kicker
                         + rob->cfg->robot_settings.kicker_thickness);
    dReal centery = y;
    dReal centerz = z - (rob->cfg->robot_settings.robot_height) * 0.5f
                      + rob->cfg->robot_settings.wheel_radius
                      - rob->cfg->robot_settings.bottom_height
                      + rob->cfg->robot_settings.kicker_z;
    box = new PBox(centerx, centery, centerz,
                   rob->cfg->robot_settings.kicker_thickness,
                   rob->cfg->robot_settings.kicker_width,
                   rob->cfg->robot_settings.kicker_height,
                   rob->cfg->robot_settings.kicker_mass, 0.9, 0.9, 0.9);
    box->setBodyPosition(centerx - x, centery - y, centerz - z, true);
    box->space = rob->space;
    rob->w->addObject(box);

    joint = dJointCreateHinge(rob->w->world, 0);
    dJointAttach(joint, rob->chassis->body, box->body);
    const dReal* aa = dBodyGetPosition(box->body);
    dJointSetHingeAnchor(joint, aa[0], aa[1], aa[2]);
    dJointSetHingeAxis(joint, 0, -1, 0);
    dJointSetHingeParam(joint, dParamVel, 0);
    dJointSetHingeParam(joint, dParamLoStop, 0);
    dJointSetHingeParam(joint, dParamHiStop, 0);
    rolling = 0;
    kicking = KickStatus::NoKick;
}

void Robot::Kicker::step() {
    if (!isTouchingBall() || rolling == 0) unholdBall();
    if (kicking != KickStatus::NoKick) {
        kickstate--;
        if (kickstate <= 0) kicking = KickStatus::NoKick;
    } else if (rolling != 0) {
        if (isTouchingBall()) holdBall();
    }
}

bool Robot::Kicker::isTouchingBall() const {
    dReal vx, vy, vz;
    dReal bx, by, bz;
    dReal kx, ky, kz;
    rob->chassis->getBodyDirection(vx, vy, vz);
    rob->getBall()->getBodyPosition(bx, by, bz);
    box->getBodyPosition(kx, ky, kz);
    kx += vx * rob->cfg->robot_settings.kicker_thickness * 0.5f;
    ky += vy * rob->cfg->robot_settings.kicker_thickness * 0.5f;
    dReal xx = fabs((kx - bx) * vx + (ky - by) * vy);
    dReal yy = fabs(-(kx - bx) * vy + (ky - by) * vx);
    dReal zz = fabs(kz - bz);
    return ((xx < rob->cfg->robot_settings.kicker_thickness * 2.0f + rob->cfg->BallRadius())
            && (yy < rob->cfg->robot_settings.kicker_width * 0.5f)
            && (zz < rob->cfg->robot_settings.kicker_height * 0.5f));
}

KickStatus Robot::Kicker::isKicking() const { return kicking; }

void Robot::Kicker::setRoller(int roller) { rolling = roller; }
int Robot::Kicker::getRoller() const { return rolling; }

void Robot::Kicker::kick(dReal kickspeedx, dReal kickspeedz) {
    dReal dx, dy, dz;
    rob->chassis->getBodyDirection(dx, dy, dz);
    dz = 0;
    unholdBall();
    if (isTouchingBall()) {
        dReal dlen = sqrt(dx * dx + dy * dy + dz * dz);
        dReal vx = dx * kickspeedx / dlen;
        dReal vy = dy * kickspeedx / dlen;
        dReal vz = kickspeedz;
        const dReal* vball = dBodyGetLinearVel(rob->getBall()->body);
        dReal vn = -(vball[0] * dx + vball[1] * dy) * rob->cfg->robot_settings.kicker_damp_factor;
        dReal vt = -(vball[0] * dy - vball[1] * dx);
        vx += vn * dx - vt * dy;
        vy += vn * dy + vt * dx;
        dBodySetLinearVel(rob->getBall()->body, vx, vy, vz);
        kicking = (kickspeedz >= 1) ? KickStatus::ChipKick : KickStatus::FlatKick;
        kickstate = 10;
    }
}

void Robot::Kicker::holdBall() {
    dReal vx, vy, vz;
    dReal bx, by, bz;
    dReal kx, ky, kz;
    rob->chassis->getBodyDirection(vx, vy, vz);
    rob->getBall()->getBodyPosition(bx, by, bz);
    box->getBodyPosition(kx, ky, kz);
    kx += vx * rob->cfg->robot_settings.kicker_thickness * 0.5f;
    ky += vy * rob->cfg->robot_settings.kicker_thickness * 0.5f;
    dReal xx = fabs((kx - bx) * vx + (ky - by) * vy);
    if (holdingBall || xx - rob->cfg->BallRadius() < 0) return;
    dBodySetLinearVel(rob->getBall()->body, 0, 0, 0);
    robot_to_ball = dJointCreateHinge(rob->getWorld()->world, 0);
    dJointAttach(robot_to_ball, box->body, rob->getBall()->body);
    holdingBall = true;
}

void Robot::Kicker::unholdBall() {
    if (holdingBall) {
        dJointDestroy(robot_to_ball);
        robot_to_ball = nullptr;
        holdingBall = false;
    }
}

Robot::Robot(PWorld* world, PBall* ball, SimConfig* _cfg,
             dReal x, dReal y, dReal z,
             dReal r, dReal g, dReal b,
             int rob_id, int dir)
    : w(world), m_ball(ball), m_x(x), m_y(y), m_z(z),
      m_r(r), m_g(g), m_b(b), m_dir(dir), m_rob_id(rob_id),
      firsttime(true), cfg(_cfg) {

    AccSpeedupAbsoluteMax = cfg->robot_settings.acc_speedup_absolute_max;
    AccSpeedupAngularMax = cfg->robot_settings.acc_speedup_angular_max;
    AccBrakeAbsoluteMax = cfg->robot_settings.acc_brake_absolute_max;
    AccBrakeAngularMax = cfg->robot_settings.acc_brake_angular_max;
    VelAbsoluteMax = cfg->robot_settings.vel_absolute_max;
    VelAngularMax = cfg->robot_settings.vel_angular_max;

    space = w->space;
    chassis = new PCylinder(x, y, z, cfg->robot_settings.robot_radius,
                            cfg->robot_settings.robot_height,
                            cfg->robot_settings.body_mass * 0.99f, r, g, b, -1, true);
    chassis->space = space;
    w->addObject(chassis);

    dummy = new PBall(x, y, z, cfg->robot_settings.robot_center_from_kicker,
                      cfg->robot_settings.body_mass * 0.01f, 0, 0, 0);
    dummy->setVisibility(false);
    dummy->space = space;
    w->addObject(dummy);

    dummy_to_chassis = dJointCreateFixed(world->world, 0);
    dJointAttach(dummy_to_chassis, chassis->body, dummy->body);

    kicker = new Kicker(this);
    wheels[0] = new Wheel(this, 0, cfg->robot_settings.wheel1_angle, cfg->robot_settings.wheel1_angle);
    wheels[1] = new Wheel(this, 1, cfg->robot_settings.wheel2_angle, cfg->robot_settings.wheel2_angle);
    wheels[2] = new Wheel(this, 2, cfg->robot_settings.wheel3_angle, cfg->robot_settings.wheel3_angle);
    wheels[3] = new Wheel(this, 3, cfg->robot_settings.wheel4_angle, cfg->robot_settings.wheel4_angle);
    on = true;
}

Robot::~Robot() {
    for (auto& wheel : wheels) delete wheel;
    delete kicker;
    // chassis and dummy deleted here; bodies already removed from world step-wise
    delete chassis;
    delete dummy;
}

PBall* Robot::getBall() const { return m_ball; }
PWorld* Robot::getWorld() const { return w; }
int Robot::getID() const { return m_rob_id - 1; }

void Robot::step() {
    if (on) {
        if (firsttime) {
            if (m_dir == -1) setDir(180);
            firsttime = false;
        }
        for (auto* wh : wheels) wh->step();
        kicker->step();
    } else if (last_state) {
        for (auto* wh : wheels) { wh->speed = 0; wh->step(); }
        kicker->setRoller(0);
        kicker->step();
    }
    last_state = on;
}

void Robot::resetSpeeds() {
    for (auto* wh : wheels) wh->speed = 0;
}

void Robot::resetRobot() {
    resetSpeeds();
    dBodySetLinearVel(chassis->body, 0, 0, 0);
    dBodySetAngularVel(chassis->body, 0, 0, 0);
    dBodySetLinearVel(dummy->body, 0, 0, 0);
    dBodySetAngularVel(dummy->body, 0, 0, 0);
    dBodySetLinearVel(kicker->box->body, 0, 0, 0);
    dBodySetAngularVel(kicker->box->body, 0, 0, 0);
    for (auto* wh : wheels) {
        dBodySetLinearVel(wh->cyl->body, 0, 0, 0);
        dBodySetAngularVel(wh->cyl->body, 0, 0, 0);
    }
    dReal x, y;
    getXY(x, y);
    setXY(x, y);
    if (m_dir == -1) setDir(180);
    else setDir(0);
}

void Robot::getXY(dReal& x, dReal& y) const {
    dReal xx, yy, zz;
    chassis->getBodyPosition(xx, yy, zz);
    x = xx; y = yy;
}

dReal Robot::getDir() const {
    dReal x, y, z;
    chassis->getBodyDirection(x, y, z);
    dReal length = sqrt(x * x + y * y);
    if (length < 1e-9) return 0;
    dReal absAng = acos(x / length) * (180.0 / M_PI);
    return (y > 0) ? absAng : -absAng;
}

dReal Robot::getDir(dReal& k) const {
    dReal x, y, z;
    chassis->getBodyDirection(x, y, z, k);
    dReal length = sqrt(x * x + y * y);
    if (length < 1e-9) return 0;
    dReal absAng = acos(x / length) * (180.0 / M_PI);
    return (y > 0) ? absAng : -absAng;
}

void Robot::setXY(dReal x, dReal y) {
    dReal xx, yy, zz, kx, ky, kz;
    dReal height = robotStartZ(cfg);
    chassis->getBodyPosition(xx, yy, zz);
    chassis->setBodyPosition(x, y, height);
    dummy->setBodyPosition(x, y, height);
    kicker->box->getBodyPosition(kx, ky, kz);
    kicker->box->setBodyPosition(kx - xx + x, ky - yy + y, kz - zz + height);
    for (auto* wh : wheels) {
        wh->cyl->getBodyPosition(kx, ky, kz);
        wh->cyl->setBodyPosition(kx - xx + x, ky - yy + y, kz - zz + height);
    }
}

void Robot::setDir(dReal ang) {
    ang *= M_PI / 180.0;
    chassis->setBodyRotation(0, 0, 1, ang);
    kicker->box->setBodyRotation(0, 0, 1, ang);
    dummy->setBodyRotation(0, 0, 1, ang);
    dMatrix3 wLocalRot, wRot, cRot;
    dVector3 localPos, finalPos, cPos;
    chassis->getBodyPosition(cPos[0], cPos[1], cPos[2], false);
    chassis->getBodyRotation(cRot, false);
    kicker->box->getBodyPosition(localPos[0], localPos[1], localPos[2], true);
    dMultiply0(finalPos, cRot, localPos, 4, 3, 1);
    finalPos[0] += cPos[0]; finalPos[1] += cPos[1]; finalPos[2] += cPos[2];
    kicker->box->setBodyPosition(finalPos[0], finalPos[1], finalPos[2], false);
    for (auto* wh : wheels) {
        wh->cyl->getBodyRotation(wLocalRot, true);
        dMultiply0(wRot, cRot, wLocalRot, 3, 3, 3);
        dBodySetRotation(wh->cyl->body, wRot);
        wh->cyl->getBodyPosition(localPos[0], localPos[1], localPos[2], true);
        dMultiply0(finalPos, cRot, localPos, 4, 3, 1);
        finalPos[0] += cPos[0]; finalPos[1] += cPos[1]; finalPos[2] += cPos[2];
        wh->cyl->setBodyPosition(finalPos[0], finalPos[1], finalPos[2], false);
    }
}

void Robot::setSpeed(int i, dReal s) {
    if (i >= 0 && i < 4) wheels[i]->speed = s;
}

void Robot::setSpeed(dReal vx, dReal vy, dReal vw) {
    const dReal DEG2RAD = M_PI / 180.0;
    dReal v = sqrt(vx * vx + vy * vy);
    if (v > VelAbsoluteMax) {
        vx *= VelAbsoluteMax / v;
        vy *= VelAbsoluteMax / v;
        v = VelAbsoluteMax;
    }
    if (fabs(vw) > VelAngularMax) vw = copysign(VelAngularMax, vw);

    const dReal* cvv = dBodyGetLinearVel(chassis->body);
    dReal cv = sqrt(cvv[0] * cvv[0] + cvv[1] * cvv[1]);
    dReal a = (v - cv) / cfg->DeltaTime() / 2;
    dReal aLimit = a > 0 ? AccSpeedupAbsoluteMax : AccBrakeAbsoluteMax;
    if (fabs(a) > aLimit) {
        a = copysign(aLimit, a);
        dReal new_v = cv + a * cfg->DeltaTime() * 2;
        if (v > 0) {
            vx *= new_v / v;
            vy *= new_v / v;
        } else if (cv > 1e-6) {
            dReal angle = getDir() * DEG2RAD;
            dReal cvx = cvv[0] * cos(angle) + cvv[1] * sin(angle);
            dReal cvy = -cvv[0] * sin(angle) + cvv[1] * cos(angle);
            vx = cvx * (new_v / cv);
            vy = cvy * (new_v / cv);
        }
    }

    const dReal* cvvw = dBodyGetAngularVel(chassis->body);
    dReal cvw = cvvw[2];
    dReal aw = (vw - cvw) / cfg->DeltaTime() / 2;
    dReal awLimit = aw > 0 ? AccSpeedupAngularMax : AccBrakeAngularMax;
    if (fabs(aw) > awLimit) {
        aw = copysign(awLimit, aw);
        vw = cvw + aw * cfg->DeltaTime() * 2;
    }

    dReal motorAlpha[4] = {
        cfg->robot_settings.wheel1_angle * DEG2RAD,
        cfg->robot_settings.wheel2_angle * DEG2RAD,
        cfg->robot_settings.wheel3_angle * DEG2RAD,
        cfg->robot_settings.wheel4_angle * DEG2RAD
    };
    dReal R = cfg->robot_settings.robot_radius;
    dReal wr = cfg->robot_settings.wheel_radius;
    for (int i = 0; i < 4; i++) {
        dReal dw = (1.0 / wr) * ((R * vw) - (vx * sin(motorAlpha[i])) + (vy * cos(motorAlpha[i])));
        setSpeed(i, dw);
    }
}

dReal Robot::getSpeed(int i) const {
    if (i < 0 || i >= 4) return -1;
    return wheels[i]->speed;
}

}  // namespace grsim
