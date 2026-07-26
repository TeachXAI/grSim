#pragma once
#include "grsim/physics/pobject.h"

namespace grsim {

class PFixedBox : public PObject {
public:
    PFixedBox(dReal x, dReal y, dReal z, dReal w, dReal h, dReal l,
              dReal r = 1, dReal g = 1, dReal b = 1);
    ~PFixedBox() override;
    void init() override;
private:
    dReal m_w, m_h, m_l;
};

}  // namespace grsim
