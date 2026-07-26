#include "grsim/physics/pground.h"

namespace grsim {

PGround::PGround(dReal field_radius, dReal field_length, dReal field_width,
                 dReal field_penalty_depth, dReal field_penalty_width,
                 dReal field_penalty_point, dReal field_line_width)
    : PObject(0, 0, 0, 0, 1, 0, 0),
      rad(field_radius), len(field_length), wid(field_width),
      pdep(field_penalty_depth), pwid(field_penalty_width),
      ppoint(field_penalty_point), lwidth(field_line_width) {}

PGround::~PGround() = default;

void PGround::init() {
    geom = dCreatePlane(space, 0, 0, 1, 0);
}

}  // namespace grsim
