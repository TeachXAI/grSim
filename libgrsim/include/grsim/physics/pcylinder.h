#pragma once
#include "grsim/physics/pobject.h"

namespace grsim {

class PCylinder : public PObject {
public:
    PCylinder(dReal x, dReal y, dReal z, dReal radius, dReal length, dReal mass,
              dReal red = 0.9, dReal green = 0.9, dReal blue = 0.9,
              int tex_id = -1, bool robot = false);
    ~PCylinder() override;
    void setMass(dReal mass) override;
    void init() override;
private:
    dReal m_radius, m_length;
    int m_texid;
    bool m_robot;
};

}  // namespace grsim
