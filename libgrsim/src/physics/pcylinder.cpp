#include "grsim/physics/pcylinder.h"

namespace grsim {

PCylinder::PCylinder(dReal x, dReal y, dReal z, dReal radius, dReal length, dReal mass,
                     dReal red, dReal green, dReal blue, int tex_id, bool robot)
    : PObject(x, y, z, red, green, blue, mass),
      m_radius(radius), m_length(length), m_texid(tex_id), m_robot(robot) {}

PCylinder::~PCylinder() = default;

void PCylinder::setMass(dReal mass) {
    m_mass = mass;
    dMass m;
    dMassSetCylinderTotal(&m, m_mass, 1, m_radius, m_length);
    dBodySetMass(body, &m);
}

void PCylinder::init() {
    body = dBodyCreate(world);
    initPosBody();
    setMass(m_mass);
    geom = dCreateCylinder(0, m_radius, m_length);
    dGeomSetBody(geom, body);
    dSpaceAdd(space, geom);
}

}  // namespace grsim
