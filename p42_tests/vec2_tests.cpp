#include "doctest.h"

#include "vec2.h"

namespace {

constexpr float pi = 3.14159265358979323846f;

void check_vec2(math::vec2 v, float expected_x, float expected_y)
{
    CHECK(v.x == doctest::Approx(expected_x));
    CHECK(v.y == doctest::Approx(expected_y));
}

} // namespace

TEST_CASE("vec2 ctor")
{
    math::vec2 v;
    
    check_vec2(v, 0.0f, 0.0f);
}

TEST_CASE("vec2 operator-(vec2 v)")
{
    math::vec2 v { 3.0f, -4.0f };
    
    check_vec2(-v, -3.0f, 4.0f);
}

TEST_CASE("vec2 operator+(vec2 u, vec2 v)")
{
    math::vec2 u { 3.0f, -4.0f };
    math::vec2 v { -1.5f, 2.0f };
    
    check_vec2(u + v, 1.5f, -2.0f);
}

TEST_CASE("vec2 operator-(vec2 u, vec2 v)")
{
    math::vec2 u { 3.0f, -4.0f };
    math::vec2 v { -1.5f, 2.0f };
    
    check_vec2(u - v, 4.5f, -6.0f);
}

TEST_CASE("vec2 operator*(vec2 v, float scalar)")
{
    math::vec2 v { 3.0f, -4.0f };
    
    check_vec2(v * 2.0f, 6.0f, -8.0f);
}

TEST_CASE("vec2 operator*(float scalar, vec2 v)")
{
    math::vec2 v { 3.0f, -4.0f };
    
    check_vec2(2.0f * v, 6.0f, -8.0f);
}

TEST_CASE("vec2 operator/(vec2 v, float scalar)")
{
    math::vec2 v { 3.0f, -4.0f };
    
    check_vec2(v / 2.0f, 1.5f, -2.0f);
}

TEST_CASE("vec2& operator+=(vec2& u, vec2 v)")
{
    math::vec2 v { 2.0f, -8.0f };
    
    v += { 3.0f, 1.0f };
    
    check_vec2(v, 5.0f, -7.0f);
}

TEST_CASE("vec2& operator-=(vec2& u, vec2 v)")
{
    math::vec2 v { 5.0f, -7.0f };
    
    v -= { 1.0f, -3.0f };
    
    check_vec2(v, 4.0f, -4.0f);
}

TEST_CASE("vec2& operator*=(vec2& v, float scalar)")
{
    math::vec2 v { 4.0f, -4.0f };
    
    v *= 0.5f;
    
    check_vec2(v, 2.0f, -2.0f);
}

TEST_CASE("vec2& operator/=(vec2& v, float scalar)")
{
    math::vec2 v { 2.0f, -2.0f };
    
    v /= 4.0f;
    
    check_vec2(v, 0.5f, -0.5f);
}

TEST_CASE("vec2 cross(float scalar, vec2 v)")
{
    math::vec2 v { 3.0f, 4.0f };
    
    check_vec2(math::cross(2.0f, v), -8.0f, 6.0f);
}

TEST_CASE("vec2 forward(float angle)")
{
    check_vec2(math::forward(0.0f),        1.0f,  0.0f);
    check_vec2(math::forward(pi / 2.0f),   0.0f,  1.0f);
    check_vec2(math::forward(pi),         -1.0f,  0.0f);
    check_vec2(math::forward(-pi / 2.0f),  0.0f, -1.0f);
}

TEST_CASE("vec2 right(float angle)")
{
    check_vec2(math::right(0.0f),         0.0f,  1.0f);
    check_vec2(math::right(pi / 2.0f),   -1.0f,  0.0f);
    check_vec2(math::right(pi),           0.0f, -1.0f);
    check_vec2(math::right(-pi / 2.0f),   1.0f,  0.0f);
}

TEST_CASE("vec2 rotate(vec2 v, float angle)")
{
    check_vec2(math::rotate({ 3.0f, -4.0f }, 0.0f),       3.0f, -4.0f);
    check_vec2(math::rotate({ 1.0f,  0.0f }, pi / 2.0f),  0.0f,  1.0f);
    check_vec2(math::rotate({ 1.0f,  0.0f }, pi),        -1.0f,  0.0f);
    check_vec2(math::rotate({ 1.0f,  0.0f }, -pi / 2.0f), 0.0f, -1.0f);
}

TEST_CASE("float cross(vec2 u, vec2 v)")
{
    math::vec2 u { 3.0f, 4.0f };
    math::vec2 v { -2.0f, 5.0f };
    
    CHECK(math::cross(u, v) == doctest::Approx(23.0f));
}

TEST_CASE("float dot(vec2 u, vec2 v)")
{
    math::vec2 u { 3.0f, 4.0f };
    math::vec2 v { -2.0f, 5.0f };
    
    CHECK(math::dot(u, v) == doctest::Approx(14.0f));
}

TEST_CASE("float length_squared(vec2 v)")
{
    CHECK(math::length_squared({  3.0f,  4.0f }) == doctest::Approx(25.0f));
    CHECK(math::length_squared({ -3.0f,  4.0f }) == doctest::Approx(25.0f));
    CHECK(math::length_squared({  3.0f, -4.0f }) == doctest::Approx(25.0f));
    CHECK(math::length_squared({ -3.0f, -4.0f }) == doctest::Approx(25.0f));
}
