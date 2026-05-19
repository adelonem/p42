#include "vec2.h"

#include <cmath>

namespace math {

vec2 operator-(vec2 v)
{
    return {
        -v.x,
        -v.y
    };
}

vec2 operator+(vec2 u, vec2 v)
{
    return {
        u.x + v.x,
        u.y + v.y
    };
}

vec2 operator-(vec2 u, vec2 v)
{
    return {
        u.x - v.x,
        u.y - v.y
    };
}

vec2 operator*(vec2 v, float scalar)
{
    return {
        v.x * scalar,
        v.y * scalar
    };
}

vec2 operator*(float scalar, vec2 v)
{
    return v * scalar;
}

vec2 operator/(vec2 v, float scalar)
{
    return {
        v.x / scalar,
        v.y / scalar
    };
}

vec2& operator+=(vec2& u, vec2 v)
{
    u.x += v.x;
    u.y += v.y;
    
    return u;
}

vec2& operator-=(vec2& u, vec2 v)
{
    u.x -= v.x;
    u.y -= v.y;
    
    return u;
}

vec2& operator*=(vec2& v, float scalar)
{
    v.x *= scalar;
    v.y *= scalar;
    
    return v;
}

vec2& operator/=(vec2& v, float scalar)
{
    v.x /= scalar;
    v.y /= scalar;
    
    return v;
}

vec2 cross(float scalar, vec2 v)
{
    return {
        -scalar * v.y,
         scalar * v.x
    };
}

vec2 forward(float angle)
{
    return {
        std::cos(angle),
        std::sin(angle)
    };
}

vec2 right(float angle)
{
    vec2 f = forward(angle);
    
    return {
        -f.y,
         f.x
    };
}

vec2 rotate(vec2 v, float angle)
{
    float c = std::cos(angle);
    float s = std::sin(angle);
    
    return {
        v.x * c - v.y * s,
        v.x * s + v.y * c
    };
}

float cross(vec2 v, vec2 u)
{
    return v.x * u.y - v.y * u.x;
}

float dot(vec2 u, vec2 v)
{
    return u.x * v.x + u.y * v.y;
}

float length_squared(vec2 v)
{
    return v.x * v.x + v.y * v.y;
}

} // namespace math
