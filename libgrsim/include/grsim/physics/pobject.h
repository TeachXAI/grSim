#pragma once

#include <ode/ode.h>

namespace grsim {

class PObject {
protected:
    dReal m_x, m_y, m_z;
    dReal m_red, m_green, m_blue;
    dReal m_mass;
    dMatrix3 local_Rot{};
    dVector3 local_Pos{};
    dQuaternion q{};
    bool isQSet = false;
    bool visible = true;
    void initPosBody();
    void initPosGeom();
public:
    PObject(dReal x, dReal y, dReal z, dReal red, dReal green, dReal blue, dReal mass);
    virtual ~PObject();
    void setRotation(dReal x_axis, dReal y_axis, dReal z_axis, dReal ang);
    void setBodyPosition(dReal x, dReal y, dReal z, bool local = false);
    void setBodyRotation(dReal x_axis, dReal y_axis, dReal z_axis, dReal ang, bool local = false);
    void getBodyPosition(dReal& x, dReal& y, dReal& z, bool local = false) const;
    void getBodyDirection(dReal& x, dReal& y, dReal& z) const;
    void getBodyDirection(dReal& x, dReal& y, dReal& z, dReal& k) const;
    void getBodyRotation(dMatrix3 r, bool local = false) const;
    void setVisibility(bool v);
    bool getVisibility() const;
    void setColor(dReal r, dReal g, dReal b);
    void getColor(dReal& r, dReal& g, dReal& b) const;
    virtual void setMass(dReal mass);
    virtual void init() = 0;

    dBodyID body = nullptr;
    dGeomID geom = nullptr;
    dWorldID world = nullptr;
    dSpaceID space = nullptr;
    int tag = 0;
    int id = 0;
};

}  // namespace grsim
