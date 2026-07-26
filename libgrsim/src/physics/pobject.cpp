#include "grsim/physics/pobject.h"
#include <cmath>

namespace grsim {

PObject::PObject(dReal x, dReal y, dReal z, dReal red, dReal green, dReal blue, dReal mass)
    : m_x(x), m_y(y), m_z(z), m_red(red), m_green(green), m_blue(blue), m_mass(mass) {
    local_Pos[0] = local_Pos[1] = local_Pos[2] = 0;
}

PObject::~PObject() {
    // ODE resources are owned by PWorld (space/world destruction).
    // Avoid double-free when wrappers are deleted after the world.
    geom = nullptr;
    body = nullptr;
}

void PObject::setVisibility(bool v) { visible = v; }
bool PObject::getVisibility() const { return visible; }

void PObject::setColor(dReal r, dReal g, dReal b) {
    m_red = r; m_green = g; m_blue = b;
}

void PObject::getColor(dReal& r, dReal& g, dReal& b) const {
    r = m_red; g = m_green; b = m_blue;
}

void PObject::setRotation(dReal x_axis, dReal y_axis, dReal z_axis, dReal ang) {
    dQFromAxisAndAngle(q, x_axis, y_axis, z_axis, ang);
    isQSet = true;
}

void PObject::setBodyPosition(dReal x, dReal y, dReal z, bool local) {
    if (!local) dBodySetPosition(body, x, y, z);
    else { local_Pos[0] = x; local_Pos[1] = y; local_Pos[2] = z; }
}

void PObject::setBodyRotation(dReal x_axis, dReal y_axis, dReal z_axis, dReal ang, bool local) {
    if (!local) {
        dQFromAxisAndAngle(q, x_axis, y_axis, z_axis, ang);
        dBodySetQuaternion(body, q);
    } else {
        dRFromAxisAndAngle(local_Rot, x_axis, y_axis, z_axis, ang);
    }
}

void PObject::getBodyPosition(dReal& x, dReal& y, dReal& z, bool local) const {
    if (local) {
        x = local_Pos[0]; y = local_Pos[1]; z = local_Pos[2];
        return;
    }
    const dReal* r = dBodyGetPosition(body);
    x = r[0]; y = r[1]; z = r[2];
}

void PObject::getBodyDirection(dReal& x, dReal& y, dReal& z) const {
    const dReal* r = dBodyGetRotation(body);
    dVector3 v = {1, 0, 0};
    dVector3 axis;
    dMultiply0(axis, r, v, 4, 3, 1);
    x = axis[0]; y = axis[1]; z = axis[2];
}

void PObject::getBodyDirection(dReal& x, dReal& y, dReal& z, dReal& k) const {
    const dReal* r = dBodyGetRotation(body);
    dVector3 v = {1, 0, 0};
    dVector3 axis;
    dMultiply0(axis, r, v, 4, 3, 1);
    x = axis[0]; y = axis[1]; z = axis[2];
    k = r[10];
}

void PObject::getBodyRotation(dMatrix3 r, bool local) const {
    if (local) {
        for (int k = 0; k < 12; k++) r[k] = local_Rot[k];
    } else {
        const dReal* rr = dBodyGetRotation(body);
        for (int k = 0; k < 12; k++) r[k] = rr[k];
    }
}

void PObject::initPosBody() {
    dBodySetPosition(body, m_x, m_y, m_z);
    if (isQSet) dBodySetQuaternion(body, q);
}

void PObject::initPosGeom() {
    dGeomSetPosition(geom, m_x, m_y, m_z);
    if (isQSet) dGeomSetQuaternion(geom, q);
}

void PObject::setMass(dReal mass) { m_mass = mass; }

}  // namespace grsim
