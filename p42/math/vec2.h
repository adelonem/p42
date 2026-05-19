#pragma once

namespace math {

struct vec2 {
    float x {};
    float y {};
};

vec2 operator-(vec2 v);
vec2 operator+(vec2 u, vec2 v);
vec2 operator-(vec2 u, vec2 v);
vec2 operator*(vec2 v, float scalar);
vec2 operator*(float scalar, vec2 v);
vec2 operator/(vec2 v, float scalar);

vec2& operator+=(vec2& u, vec2 v);
vec2& operator-=(vec2& u, vec2 v);
vec2& operator*=(vec2& v, float scalar);
vec2& operator/=(vec2& v, float scalar);

// Returns the 2D cross product between a scalar and a vector.
[[nodiscard]] vec2 cross(float scalar, vec2 v);
[[nodiscard]] vec2 forward(float angle);
[[nodiscard]] vec2 right(float angle);
[[nodiscard]] vec2 rotate(vec2 v, float angle);

[[nodiscard]] float cross(vec2 u, vec2 v);
[[nodiscard]] float dot(vec2 u, vec2 v);
[[nodiscard]] float length_squared(vec2 v);

} // namespace math
