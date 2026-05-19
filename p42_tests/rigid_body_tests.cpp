#include "doctest.h"

#include "../../p42/p42/physics/rigid_body.h"

#include <cmath>

namespace {

constexpr float pi = 3.14159265358979323846f;

void check_vec2(math::vec2 v, float expected_x, float expected_y)
{
    CHECK(v.x == doctest::Approx(expected_x));
    CHECK(v.y == doctest::Approx(expected_y));
}

} // namespace

TEST_CASE("rigid_body ctor")
{
    physics::rigid_body b;
    
    check_vec2(b.position, 0.0f, 0.0f);
    check_vec2(b.velocity, 0.0f, 0.0f);
    check_vec2(b.force,    0.0f, 0.0f);
    CHECK(b.angle            == doctest::Approx(0.0f));
    CHECK(b.angular_velocity == doctest::Approx(0.0f));
    CHECK(b.torque           == doctest::Approx(0.0f));
    CHECK(b.mass             == doctest::Approx(1.0f));
    CHECK(b.inertia          == doctest::Approx(1.0f));
    CHECK(b.damping          == doctest::Approx(0.0f));
    CHECK(b.angular_damping  == doctest::Approx(0.0f));
    CHECK(b.restitution      == doctest::Approx(0.0f));
}

TEST_CASE("void apply_force(rigid_body& body, math::vec2 force)")
{
    physics::rigid_body b;
    
    physics::apply_force(b, { 3.0f, -2.0f });
    check_vec2(b.force, 3.0f, -2.0f);
    
    physics::apply_force(b, { 1.0f, 5.0f });
    check_vec2(b.force, 4.0f, 3.0f);
    
    CHECK(b.torque == doctest::Approx(0.0f));
}

TEST_CASE("void apply_force_at(rigid_body& body, math::vec2 force, math::vec2 offset)")
{
    physics::rigid_body b;
    physics::apply_force_at(b, { 1.0f, 0.0f }, { 0.0f, 1.0f });
    check_vec2(b.force, 1.0f, 0.0f);
    CHECK(b.torque == doctest::Approx(-1.0f));
    
    physics::rigid_body c;
    physics::apply_force_at(c, { 5.0f, 3.0f }, { 0.0f, 0.0f });
    check_vec2(c.force, 5.0f, 3.0f);
    CHECK(c.torque == doctest::Approx(0.0f));
}

TEST_CASE("void apply_local_force(rigid_body& body, math::vec2 force)")
{
    // angle = 0: equivalent to apply_force.
    physics::rigid_body b;
    physics::apply_local_force(b, { 3.0f, 4.0f });
    check_vec2(b.force, 3.0f, 4.0f);
    
    // angle = pi/2: force is rotated by 90 degrees.
    physics::rigid_body c;
    c.angle = pi / 2.0f;
    physics::apply_local_force(c, { 1.0f, 0.0f });
    check_vec2(c.force, 0.0f, 1.0f);
}

TEST_CASE("void apply_local_force_at(rigid_body& body, math::vec2 force, math::vec2 offset)")
{
    // angle = 0: equivalent to apply_force_at.
    physics::rigid_body b;
    physics::apply_local_force_at(b, { 1.0f, 0.0f }, { 0.0f, 1.0f });
    
    // cross(offset, force) = 0*0 - 1*1 = -1
    check_vec2(b.force, 1.0f, 0.0f);
    CHECK(b.torque == doctest::Approx(-1.0f));
    
    // angle = pi/2: both force and offset are rotated.
    physics::rigid_body c;
    c.angle = pi / 2.0f;
    physics::apply_local_force_at(c, { 1.0f, 0.0f }, { 1.0f, 0.0f });
    
    // rotated force  = rotate({1, 0}, pi/2) = {0, 1}
    // rotated offset = rotate({1, 0}, pi/2) = {0, 1}
    // cross(offset, force) = 0*1 - 1*0 = 0
    check_vec2(c.force, 0.0f, 1.0f);
    CHECK(c.torque == doctest::Approx(0.0f));
}

TEST_CASE("void apply_torque(rigid_body& body, float torque)")
{
    physics::rigid_body b;
    
    physics::apply_torque(b, 1.5f);
    CHECK(b.torque == doctest::Approx(1.5f));
    
    physics::apply_torque(b, -0.5f);
    CHECK(b.torque == doctest::Approx(1.0f));
}

TEST_CASE("void step(rigid_body& body, float dt)")
{
    // Integrates velocity from force (semi-implicit Euler).
    {
        physics::rigid_body b;
        physics::apply_force(b, { 2.0f, 0.0f });
        
        physics::step(b, 1.0f);
        
        check_vec2(b.velocity, 2.0f, 0.0f);
        check_vec2(b.position, 2.0f, 0.0f);
    }
    
    // Integrates position from initial velocity (no force).
    {
        physics::rigid_body b;
        b.velocity = { 3.0f, 0.0f };
        
        physics::step(b, 0.5f);
        
        check_vec2(b.velocity, 3.0f, 0.0f);
        check_vec2(b.position, 1.5f, 0.0f);
    }
    
    // Integrates angular velocity from torque.
    {
        physics::rigid_body b;
        physics::apply_torque(b, 3.0f);
        
        physics::step(b, 1.0f);
        
        CHECK(b.angular_velocity == doctest::Approx(3.0f));
        CHECK(b.angle            == doctest::Approx(3.0f));
    }
    
    // Resets force and torque after step.
    {
        physics::rigid_body b;
        physics::apply_force(b, { 1.0f, 1.0f });
        physics::apply_torque(b, 0.5f);
        
        physics::step(b, 0.1f);
        
        check_vec2(b.force, 0.0f, 0.0f);
        CHECK(b.torque == doctest::Approx(0.0f));
    }
    
    // mass <= 0: body is immovable in translation.
    {
        physics::rigid_body b;
        b.mass = 0.0f;
        physics::apply_force(b, { 10.0f, 5.0f });
        
        physics::step(b, 1.0f);
        
        check_vec2(b.velocity, 0.0f, 0.0f);
        check_vec2(b.position, 0.0f, 0.0f);
    }
    
    // inertia <= 0: body is immovable in rotation.
    {
        physics::rigid_body b;
        b.inertia = 0.0f;
        physics::apply_torque(b, 10.0f);
        
        physics::step(b, 1.0f);
        
        CHECK(b.angular_velocity == doctest::Approx(0.0f));
        CHECK(b.angle            == doctest::Approx(0.0f));
    }
    
    // mass scales acceleration.
    {
        physics::rigid_body b;
        b.mass = 2.0f;
        physics::apply_force(b, { 4.0f, 0.0f });
        
        physics::step(b, 1.0f);
        
        check_vec2(b.velocity, 2.0f, 0.0f);
    }
    
    // inertia scales angular acceleration.
    {
        physics::rigid_body b;
        b.inertia = 4.0f;
        physics::apply_torque(b, 8.0f);
        
        physics::step(b, 1.0f);
        
        CHECK(b.angular_velocity == doctest::Approx(2.0f));
    }
    
    // linear damping: exponential decay.
    {
        physics::rigid_body b;
        b.velocity = { 10.0f, 0.0f };
        b.damping  = 1.0f;
        
        physics::step(b, 1.0f);
        
        CHECK(b.velocity.x == doctest::Approx(10.0f * std::exp(-1.0f)));
        CHECK(b.velocity.y == doctest::Approx(0.0f));
    }
    
    // angular damping: exponential decay.
    {
        physics::rigid_body b;
        b.angular_velocity = 5.0f;
        b.angular_damping  = 2.0f;
        
        physics::step(b, 0.5f);
        
        CHECK(b.angular_velocity == doctest::Approx(5.0f * std::exp(-1.0f)));
    }
    
    // Force does not affect angular state.
    {
        physics::rigid_body b;
        physics::apply_force(b, { 5.0f, 0.0f });
        
        physics::step(b, 1.0f);
        
        CHECK(b.angular_velocity == doctest::Approx(0.0f));
        CHECK(b.angle            == doctest::Approx(0.0f));
    }
    
    // Torque does not affect linear state.
    {
        physics::rigid_body b;
        physics::apply_torque(b, 5.0f);
        
        physics::step(b, 1.0f);
        
        check_vec2(b.velocity, 0.0f, 0.0f);
        check_vec2(b.position, 0.0f, 0.0f);
    }
}
