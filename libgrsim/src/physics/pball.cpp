#include "grsim/physics/pball.h"

namespace grsim {

PBall::PBall(dReal x, dReal y, dReal z, dReal radius, dReal mass,
             dReal red, dReal green, dReal blue)
    : PObject(x, y, z, red, green, blue, mass), m_radius(radius) {}

PBall::~PBall() = default;

void PBall::init() {
    body = dBodyCreate(world);
    initPosBody();
    setMass(m_mass);
    geom = dCreateSphere(0, m_radius);
    dGeomSetBody(geom, body);
    dSpaceAdd(space, geom);
}

void PBall::setMass(dReal mass) {
    m_mass = mass;
    dMass m;
    dMassSetSphereTotal(&m, m_mass, m_radius);
    dBodySetMass(body, &m);
}

}  // namespace grsim
