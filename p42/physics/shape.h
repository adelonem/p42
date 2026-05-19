#pragma once

#include "vec2.h"

namespace physics {

struct box {
    math::vec2 center       {};
    float      angle        {};
    math::vec2 half_extents {};
};

} // namespace physics
