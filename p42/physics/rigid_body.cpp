#include "rigid_body.h"

#include <cmath>

namespace physics {

namespace {

float inverse(float value)
{
    return value > 0.0f ? 1.0f / value : 0.0f;
}

math::vec2 next_velocity(
    math::vec2 velocity,
    math::vec2 acceleration,
    float damping,
    float dt
)
{
    return (velocity + acceleration * dt) * std::exp(-damping * dt);
}

math::vec2 next_position(
    math::vec2 position,
    math::vec2 velocity,
    float dt
)
{
    return position + velocity * dt;
}

float next_angular_velocity(
    float angular_velocity,
    float angular_acceleration,
    float damping,
    float dt
)
{
    return (angular_velocity + angular_acceleration * dt) * std::exp(-damping * dt);
}

float next_angle(
    float angle,
    float angular_velocity,
    float dt
)
{
    return angle + angular_velocity * dt;
}

} // namespace

void apply_force(rigid_body& body, math::vec2 force)
{
    body.force += force;
}

void apply_force_at(rigid_body& body, math::vec2 force, math::vec2 offset)
{
    body.force += force;
    body.torque += math::cross(offset, force);
}

void apply_local_force(rigid_body& body, math::vec2 force)
{
    apply_force(body, math::rotate(force, body.angle));
}

void apply_local_force_at(rigid_body& body, math::vec2 force, math::vec2 offset)
{
    apply_force_at(
        body,
        math::rotate(force, body.angle),
        math::rotate(offset, body.angle)
    );
}

void apply_torque(rigid_body& body, float torque)
{
    body.torque += torque;
}

void step(rigid_body& body, float dt)
{
    float inv_mass = inverse(body.mass);
    math::vec2 acceleration = body.force * inv_mass;
    body.velocity = next_velocity(body.velocity, acceleration, body.damping, dt);
    body.position = next_position(body.position, body.velocity, dt);
    
    float inv_inertia = inverse(body.inertia);
    float angular_acceleration = body.torque * inv_inertia;
    body.angular_velocity = next_angular_velocity(body.angular_velocity, angular_acceleration, body.angular_damping, dt);
    body.angle = next_angle(body.angle, body.angular_velocity, dt);
    
    body.force = {};
    body.torque = 0.0f;
}

} // namespace physics
