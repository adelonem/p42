#pragma once

#include "vec2.h"

namespace physics {

struct rigid_body {
    // Linear motion.
    math::vec2 position         {};
    math::vec2 velocity         {};
    math::vec2 force            {};
    float      mass             { 1.0f }; // Values <= 0 are treated as immovable for force/impulse response.
    float      damping          {};
    
    // Angular motion.
    float      angle            {};
    float      angular_velocity {};
    float      torque           {};
    float      inertia          { 1.0f };
    float      angular_damping  {};
    
    // Material.
    float      restitution      {};
};

void apply_force(rigid_body& body, math::vec2 force);
void apply_force_at(rigid_body& body, math::vec2 force, math::vec2 offset);
void apply_local_force(rigid_body& body, math::vec2 force);
void apply_local_force_at(rigid_body& body, math::vec2 force, math::vec2 offset);
void apply_torque(rigid_body& body, float torque);
void step(rigid_body& body, float dt);

} // namespace physics
