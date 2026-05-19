#include "world.h"

#include "collision.h"
#include "vec2.h"

namespace physics {

namespace {

constexpr float penetration_slop   { 0.01f };
constexpr float correction_percent { 0.20f };

struct contact {
    rigid_body* a   {};
    rigid_body* b   {};
    collision   hit {};
};

float inverse(float value)
{
    return value > 0.0f ? 1.0f / value : 0.0f;
}

box world_box(const rigid_body& state, const collider& c)
{
    return {
        .center       = state.position + math::rotate(c.local_box.center, state.angle),
        .angle        = state.angle + c.local_box.angle,
        .half_extents = c.local_box.half_extents
    };
}

void apply_normal_impulse(rigid_body& a, rigid_body& b, collision c)
{
    float restitution = std::max(a.restitution, b.restitution);

    float inv_mass_a = inverse(a.mass);
    float inv_mass_b = inverse(b.mass);
    float inv_inertia_a = inverse(a.inertia);
    float inv_inertia_b = inverse(b.inertia);
    float inv_mass_sum = inv_mass_a + inv_mass_b;

    if (inv_mass_sum <= 0.0f) {
        return;
    }

    math::vec2 ra = c.contact - a.position;
    math::vec2 rb = c.contact - b.position;

    math::vec2 contact_velocity_a = a.velocity + math::cross(a.angular_velocity, ra);
    math::vec2 contact_velocity_b = b.velocity + math::cross(b.angular_velocity, rb);

    math::vec2 relative_velocity = contact_velocity_a - contact_velocity_b;
    float velocity_along_normal = math::dot(relative_velocity, c.normal);

    if (velocity_along_normal > 0.0f) {
        return;
    }

    float ra_cross_n = math::cross(ra, c.normal);
    float rb_cross_n = math::cross(rb, c.normal);
    float effective_mass = inv_mass_sum
                         + ra_cross_n * ra_cross_n * inv_inertia_a
                         + rb_cross_n * rb_cross_n * inv_inertia_b;

    if (effective_mass <= 0.0f) {
        return;
    }

    float j = -(1.0f + restitution) * velocity_along_normal / effective_mass;
    math::vec2 impulse = c.normal * j;

    a.velocity += impulse * inv_mass_a;
    b.velocity -= impulse * inv_mass_b;
    a.angular_velocity += math::cross(ra, impulse) * inv_inertia_a;
    b.angular_velocity -= math::cross(rb, impulse) * inv_inertia_b;
}

void correct_positions(rigid_body& a, rigid_body& b, collision c)
{
    float inv_mass_a = inverse(a.mass);
    float inv_mass_b = inverse(b.mass);
    float inv_mass_sum = inv_mass_a + inv_mass_b;

    if (inv_mass_sum <= 0.0f) {
        return;
    }

    float depth = std::max(c.depth - penetration_slop, 0.0f);
    math::vec2 correction = c.normal * (depth * correction_percent / inv_mass_sum);
    a.position += correction * inv_mass_a;
    b.position -= correction * inv_mass_b;
}

void solve_contacts(
    const std::vector<contact>& contacts,
    int velocity_iterations,
    int position_iterations
)
{
    for (int iteration = 0; iteration < velocity_iterations; ++iteration) {
        for (const contact& c : contacts) {
            apply_normal_impulse(*c.a, *c.b, c.hit);
        }
    }

    for (int iteration = 0; iteration < position_iterations; ++iteration) {
        for (const contact& c : contacts) {
            correct_positions(*c.a, *c.b, c.hit);
        }
    }
}

} // namespace

bool operator==(body_handle a, body_handle b)
{
    return a.index == b.index && a.generation == b.generation;
}

bool operator!=(body_handle a, body_handle b)
{
    return !(a == b);
}

body_handle add_body(world& w, rigid_body state)
{
    if (!w.free_body_slots.empty()) {
        std::uint32_t index = w.free_body_slots.back();
        w.free_body_slots.pop_back();

        body_slot& slot = w.bodies[index];
        slot.state = state;
        slot.alive = true;
        return { index, slot.generation };
    }

    std::uint32_t index = static_cast<std::uint32_t>(w.bodies.size());
    w.bodies.push_back({ state, 1u, true });
    return { index, 1u };
}

void remove_body(world& w, body_handle h)
{
    if (h.index >= w.bodies.size()) {
        return;
    }

    body_slot& slot = w.bodies[h.index];
    if (slot.generation != h.generation || !slot.alive) {
        return;
    }

    slot.alive = false;
    slot.generation += 1; // Outstanding handles to this slot now fail validation.
    w.free_body_slots.push_back(h.index);

    w.colliders.erase(
        std::remove_if(
            w.colliders.begin(),
            w.colliders.end(),
            [h](const collider& c) {
                return c.attached.index == h.index
                    && c.attached.generation == h.generation;
            }
        ),
        w.colliders.end()
    );
}

void add_collider(world& w, body_handle attached, box local_box)
{
    w.colliders.push_back({ attached, local_box });
}

rigid_body* body_at(world& w, body_handle h)
{
    if (h.index >= w.bodies.size()) {
        return nullptr;
    }

    body_slot& slot = w.bodies[h.index];
    if (slot.generation != h.generation || !slot.alive) {
        return nullptr;
    }

    return &slot.state;
}

const rigid_body* body_at(const world& w, body_handle h)
{
    if (h.index >= w.bodies.size()) {
        return nullptr;
    }

    const body_slot& slot = w.bodies[h.index];
    if (slot.generation != h.generation || !slot.alive) {
        return nullptr;
    }

    return &slot.state;
}

void step(world& w, float dt)
{
    for (body_slot& slot : w.bodies) {
        if (!slot.alive) {
            continue;
        }
        step(slot.state, dt);
    }

    w.contact_events.clear();
    std::vector<contact> contacts {};

    for (std::size_t i = 0; i < w.colliders.size(); ++i) {
        const collider& a = w.colliders[i];
        rigid_body* body_a = body_at(w, a.attached);
        if (!body_a) {
            continue;
        }

        for (std::size_t j = i + 1; j < w.colliders.size(); ++j) {
            const collider& b = w.colliders[j];
            rigid_body* body_b = body_at(w, b.attached);
            if (!body_b) {
                continue;
            }

            if (auto hit = intersect(world_box(*body_a, a), world_box(*body_b, b))) {
                contacts.push_back({ body_a, body_b, *hit });
                w.contact_events.push_back({ a.attached, b.attached });
            }
        }
    }

    solve_contacts(contacts, w.velocity_iterations, w.position_iterations);
}

} // namespace physics
