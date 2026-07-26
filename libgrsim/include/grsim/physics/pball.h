#pragma once
#include "grsim/physics/pobject.h"

namespace grsim {

class PBall : public PObject {
public:
    PBall(dReal x, dReal y, dReal z, dReal radius, dReal mass,
          dReal red = 1, dReal green = 0.7, dReal blue = 0);
    ~PBall() override;
    void setMass(dReal mass) override;
    void init() override;
    dReal radius() const { return m_radius; }
private:
    dReal m_radius;
};

}  // namespace grsim
