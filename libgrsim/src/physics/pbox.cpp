#include "grsim/physics/pbox.h"

namespace grsim {

PBox::PBox(dReal x, dReal y, dReal z, dReal w, dReal h, dReal l, dReal mass,
           dReal r, dReal g, dReal b)
    : PObject(x, y, z, r, g, b, mass), m_w(w), m_h(h), m_l(l) {}

PBox::~PBox() = default;

void PBox::init() {
    body = dBodyCreate(world);
    initPosBody();
    setMass(m_mass);
    geom = dCreateBox(0, m_w, m_h, m_l);
    dGeomSetBody(geom, body);
    dSpaceAdd(space, geom);
}

void PBox::setMass(dReal mass) {
    m_mass = mass;
    dMass m;
    dMassSetBoxTotal(&m, m_mass, m_w, m_h, m_l);
    dBodySetMass(body, &m);
}

}  // namespace grsim
