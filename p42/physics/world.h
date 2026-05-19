#pragma once

#include <vector>

#include "rigid_body.h"
#include "shape.h"

namespace physics {

// Stable, generational reference to a body stored in a world.
// A default-constructed handle (generation 0) is always invalid:
// live slots start at generation 1 and bump on each removal.
struct body_handle {
    std::uint32_t index      {};
    std::uint32_t generation {};
};

[[nodiscard]] bool operator==(body_handle a, body_handle b);
[[nodiscard]] bool operator!=(body_handle a, body_handle b);

// A body can carry several colliders (composable shapes).
// Colliders die with their attached body — no separate lifetime to manage.
struct collider {
    body_handle attached  {};
    box         local_box {};
};

// Recorded once per detected overlap during the last `step`. Cleared at the
// start of every step. If two bodies carry multiple overlapping colliders,
// one event is emitted per colliding pair.
struct contact_event {
    body_handle a {};
    body_handle b {};
};

// Slot-based storage so body addresses (and handles) stay stable across
// add/remove operations. A free list reuses slots without invalidating
// live handles thanks to the per-slot generation counter.
struct body_slot {
    rigid_body    state      {};
    std::uint32_t generation {};
    bool          alive      {};
};

struct world {
    std::vector<body_slot>     bodies          {};
    std::vector<collider>      colliders       {};
    std::vector<std::uint32_t> free_body_slots {};

    // Populated by step(), readable afterwards. Each entry refers to a pair of
    // bodies that overlapped this frame.
    std::vector<contact_event> contact_events  {};

    int velocity_iterations { 8 };
    int position_iterations { 4 };
};

body_handle add_body(world& w, rigid_body state);
void        remove_body(world& w, body_handle h);
void        add_collider(world& w, body_handle attached, box local_box);

[[nodiscard]] rigid_body*       body_at(world& w, body_handle h);
[[nodiscard]] const rigid_body* body_at(const world& w, body_handle h);

void step(world& w, float dt);

} // namespace physics
