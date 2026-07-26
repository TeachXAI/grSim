#pragma once

#include "grsim/physics/pworld.h"
#include "grsim/physics/pcylinder.h"
#include "grsim/physics/pbox.h"
#include "grsim/physics/pball.h"
#include "grsim/config.h"
#include "grsim/constants.h"

namespace grsim {

inline double robotStartZ(const SimConfig* cfg) {
    return cfg->robot_settings.robot_height * 0.5
         + cfg->robot_settings.wheel_radius * 1.1
         + cfg->robot_settings.bottom_height;
}

class Robot {
    PWorld* w;
    PBall* m_ball;
    dReal m_x, m_y, m_z;
    dReal m_r, m_g, m_b;
    dReal m_dir;
    int m_rob_id;
    bool firsttime;
    bool last_state = false;

    dReal AccSpeedupAbsoluteMax;
    dReal AccSpeedupAngularMax;
    dReal AccBrakeAbsoluteMax;
    dReal AccBrakeAngularMax;
    dReal VelAbsoluteMax;
    dReal VelAngularMax;

public:
    SimConfig* cfg;
    dSpaceID space;
    PCylinder* chassis;
    PBall* dummy;
    dJointID dummy_to_chassis;
    bool on = true;

    class Wheel {
    public:
        int id;
        Wheel(Robot* robot, int _id, dReal ang, dReal ang2);
        void step();
        dJointID joint;
        dJointID motor;
        PCylinder* cyl;
        dReal speed = 0;
        Robot* rob;
    } *wheels[4]{};

    class Kicker {
    private:
        KickStatus kicking = KickStatus::NoKick;
        int rolling = 0;
        int kickstate = 0;
        bool holdingBall = false;
    public:
        explicit Kicker(Robot* robot);
        void step();
        void kick(dReal kickspeedx, dReal kickspeedz);
        void setRoller(int roller);
        int getRoller() const;
        bool isTouchingBall() const;
        KickStatus isKicking() const;
        void holdBall();
        void unholdBall();
        dJointID joint;
        dJointID robot_to_ball = nullptr;
        PBox* box;
        Robot* rob;
    } *kicker = nullptr;

    Robot(PWorld* world, PBall* ball, SimConfig* _cfg,
          dReal x, dReal y, dReal z,
          dReal r, dReal g, dReal b,
          int rob_id, int dir);
    ~Robot();

    void step();
    void setSpeed(int i, dReal s);
    void setSpeed(dReal vx, dReal vy, dReal vw);
    dReal getSpeed(int i) const;
    void resetSpeeds();
    void resetRobot();
    void getXY(dReal& x, dReal& y) const;
    dReal getDir() const;
    dReal getDir(dReal& k) const;
    void setXY(dReal x, dReal y);
    void setDir(dReal ang);
    int getID() const;
    PBall* getBall() const;
    PWorld* getWorld() const;
};

}  // namespace grsim
