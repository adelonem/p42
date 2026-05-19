#pragma once

#include <optional>

#include "shape.h"
#include "vec2.h"

namespace physics {

struct collision {
    math::vec2 contact {}; // World-space contact point.
    math::vec2 normal  {}; // Unit vector pointing from b toward a.
    float      depth   {}; // Overlap along the normal.
};

[[nodiscard]] std::optional<collision> intersect(const box& a, const box& b);

} // namespace physics
