#include "grsim/physics/pfixedbox.h"

namespace grsim {

PFixedBox::PFixedBox(dReal x, dReal y, dReal z, dReal w, dReal h, dReal l,
                     dReal r, dReal g, dReal b)
    : PObject(x, y, z, r, g, b, 0), m_w(w), m_h(h), m_l(l) {}

PFixedBox::~PFixedBox() = default;

void PFixedBox::init() {
    geom = dCreateBox(space, m_w, m_h, m_l);
    initPosGeom();
}

}  // namespace grsim
