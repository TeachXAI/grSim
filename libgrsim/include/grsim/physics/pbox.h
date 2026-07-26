#pragma once
#include "grsim/physics/pobject.h"

namespace grsim {

class PBox : public PObject {
public:
    PBox(dReal x, dReal y, dReal z, dReal w, dReal h, dReal l, dReal mass,
         dReal r = 0.9, dReal g = 0.9, dReal b = 0.9);
    ~PBox() override;
    void setMass(dReal mass) override;
    void init() override;
private:
    dReal m_w, m_h, m_l;
};

}  // namespace grsim
