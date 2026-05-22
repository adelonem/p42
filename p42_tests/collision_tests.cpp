#include "doctest.h"

#include "collision.h"

namespace {

constexpr float pi = 3.14159265358979323846f;

void check_vec2(math::vec2 v, float expected_x, float expected_y)
{
    CHECK(v.x == doctest::Approx(expected_x));
    CHECK(v.y == doctest::Approx(expected_y));
}

} // namespace

TEST_CASE("intersect: disjoint boxes return nullopt")
{
    physics::box a { .center = {  0.0f, 0.0f }, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
    physics::box b { .center = { 10.0f, 0.0f }, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
    
    CHECK_FALSE(physics::intersect(a, b).has_value());
}

TEST_CASE("intersect: boxes touching edge-to-edge return nullopt")
{
    // Distance between centers = 2.0, sum of half-extents on x = 2.0 -> overlap = 0.
    physics::box a { .center = { 0.0f, 0.0f }, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
    physics::box b { .center = { 2.0f, 0.0f }, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
    
    CHECK_FALSE(physics::intersect(a, b).has_value());
}

TEST_CASE("intersect: identical boxes overlap fully")
{
    physics::box a { .center = { 0.0f, 0.0f }, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
    physics::box b { .center = { 0.0f, 0.0f }, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
    
    auto hit = physics::intersect(a, b);
    REQUIRE(hit.has_value());
    CHECK(hit->depth == doctest::Approx(2.0f));
}

TEST_CASE("intersect: AABB overlap along x")
{
    // Centers at distance 1.5, sum of half-extents = 2.0 -> overlap = 0.5 along x.
    physics::box a { .center = { 0.0f, 0.0f }, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
    physics::box b { .center = { 1.5f, 0.0f }, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
    
    auto hit = physics::intersect(a, b);
    REQUIRE(hit.has_value());
    CHECK(hit->depth == doctest::Approx(0.5f));
    
    // Normal points from b toward a -> negative x.
    check_vec2(hit->normal, -1.0f, 0.0f);
}

TEST_CASE("intersect: AABB overlap along y")
{
    physics::box a { .center = { 0.0f, 0.0f }, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
    physics::box b { .center = { 0.0f, 1.5f }, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
    
    auto hit = physics::intersect(a, b);
    REQUIRE(hit.has_value());
    CHECK(hit->depth == doctest::Approx(0.5f));
    check_vec2(hit->normal, 0.0f, -1.0f);
}

TEST_CASE("intersect: picks axis with minimum overlap")
{
    // 0.2 overlap along x, 0.4 overlap along y -> x axis chosen.
    physics::box a { .center = { 0.0f, 0.0f }, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
    physics::box b { .center = { 1.8f, 1.6f }, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
    
    auto hit = physics::intersect(a, b);
    REQUIRE(hit.has_value());
    CHECK(hit->depth == doctest::Approx(0.2f));
    check_vec2(hit->normal, -1.0f, 0.0f);
}

TEST_CASE("intersect: rotated box overlaps AABB")
{
    // 45-degree rotated unit half-extents box has projected half-width = sqrt(2) ~ 1.414 on world axes.
    physics::box a { .center = { 0.0f, 0.0f }, .angle = 0.0f,      .half_extents = { 1.0f, 1.0f } };
    physics::box b { .center = { 2.0f, 0.0f }, .angle = pi / 4.0f, .half_extents = { 1.0f, 1.0f } };
    
    auto hit = physics::intersect(a, b);
    REQUIRE(hit.has_value());
    
    // Expected overlap on world x = 1 + sqrt(2) - 2.
    float expected = 1.0f + 1.41421356f - 2.0f;
    CHECK(hit->depth == doctest::Approx(expected));
}

TEST_CASE("intersect: rotated box separated from AABB")
{
    // Same setup but with centers far enough apart to clear the rotated projection.
    physics::box a { .center = { 0.0f, 0.0f }, .angle = 0.0f,      .half_extents = { 1.0f, 1.0f } };
    physics::box b { .center = { 3.0f, 0.0f }, .angle = pi / 4.0f, .half_extents = { 1.0f, 1.0f } };
    
    CHECK_FALSE(physics::intersect(a, b).has_value());
}

TEST_CASE("intersect: normal points from b toward a")
{
    // a is to the right of b on x -> normal should point in +x direction.
    physics::box a { .center = { 1.5f, 0.0f }, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
    physics::box b { .center = { 0.0f, 0.0f }, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
    
    auto hit = physics::intersect(a, b);
    REQUIRE(hit.has_value());
    check_vec2(hit->normal, 1.0f, 0.0f);
}

TEST_CASE("intersect: contact point of identical boxes is at shared center")
{
    physics::box a { .center = { 5.0f, -3.0f }, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
    physics::box b { .center = { 5.0f, -3.0f }, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
    
    auto hit = physics::intersect(a, b);
    REQUIRE(hit.has_value());
    check_vec2(hit->contact, 5.0f, -3.0f);
}

TEST_CASE("intersect: contact point lies inside both boxes for partial AABB overlap")
{
    physics::box a { .center = { 0.0f, 0.0f }, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
    physics::box b { .center = { 1.5f, 0.0f }, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
    
    auto hit = physics::intersect(a, b);
    REQUIRE(hit.has_value());
    
    // Overlap region is x in [0.5, 1.0], y in [-1, 1] -> center at (0.75, 0).
    check_vec2(hit->contact, 0.75f, 0.0f);
}

TEST_CASE("intersect: degenerate zero-size boxes do not collide")
{
    physics::box a { .center = { 0.0f, 0.0f }, .angle = 0.0f, .half_extents = { 0.0f, 0.0f } };
    physics::box b { .center = { 0.0f, 0.0f }, .angle = 0.0f, .half_extents = { 0.0f, 0.0f } };
    
    CHECK_FALSE(physics::intersect(a, b).has_value());
}
