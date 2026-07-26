#pragma once
#include "grsim/physics/pobject.h"

namespace grsim {

class PGround : public PObject {
public:
    PGround(dReal field_radius, dReal field_length, dReal field_width,
            dReal field_penalty_depth, dReal field_penalty_width,
            dReal field_penalty_point, dReal field_line_width);
    ~PGround() override;
    void init() override;
private:
    dReal rad, len, wid, pdep, pwid, ppoint, lwidth;
};

}  // namespace grsim
