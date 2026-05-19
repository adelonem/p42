#include "doctest.h"

#include "../../p42/p42/physics/world.h"

namespace {

void check_vec2(math::vec2 v, float expected_x, float expected_y)
{
    CHECK(v.x == doctest::Approx(expected_x));
    CHECK(v.y == doctest::Approx(expected_y));
}

} // namespace

TEST_CASE("body_handle ctor")
{
    physics::body_handle h;
    
    CHECK(h.index      == 0u);
    CHECK(h.generation == 0u);
}

TEST_CASE("bool operator==(body_handle a, body_handle b)")
{
    physics::body_handle a { 1u, 1u };
    physics::body_handle b { 1u, 1u };
    physics::body_handle c { 1u, 2u };
    physics::body_handle d { 2u, 1u };
    
    CHECK(a == b);
    CHECK_FALSE(a == c);
    CHECK_FALSE(a == d);
}

TEST_CASE("bool operator!=(body_handle a, body_handle b)")
{
    physics::body_handle a { 1u, 1u };
    physics::body_handle b { 1u, 1u };
    physics::body_handle c { 1u, 2u };
    
    CHECK_FALSE(a != b);
    CHECK(a != c);
}

TEST_CASE("body_handle add_body(world& w, rigid_body state)")
{
    // Returns a valid handle resolving to the stored state.
    {
        physics::world w;
        physics::rigid_body state;
        state.position = { 1.0f, 2.0f };
        
        physics::body_handle h = physics::add_body(w, state);
        
        REQUIRE(h.generation != 0u);
        
        physics::rigid_body* body = physics::body_at(w, h);
        REQUIRE(body != nullptr);
        check_vec2(body->position, 1.0f, 2.0f);
    }
    
    // Successive calls return distinct handles.
    {
        physics::world w;
        physics::body_handle h1 = physics::add_body(w, {});
        physics::body_handle h2 = physics::add_body(w, {});
        
        CHECK(h1 != h2);
    }
}

TEST_CASE("void remove_body(world& w, body_handle h)")
{
    // Removing a body invalidates its handle.
    {
        physics::world w;
        physics::body_handle h = physics::add_body(w, {});
        REQUIRE(physics::body_at(w, h) != nullptr);
        
        physics::remove_body(w, h);
        
        CHECK(physics::body_at(w, h) == nullptr);
    }
    
    // Removing with an out-of-range or stale handle is a no-op.
    {
        physics::world w;
        physics::body_handle h = physics::add_body(w, {});
        
        physics::remove_body(w, { 99u, 1u });
        physics::remove_body(w, { h.index, h.generation + 1u });
        
        CHECK(physics::body_at(w, h) != nullptr);
    }
    
    // Slots are reused, but the generation bump invalidates stale handles.
    {
        physics::world w;
        physics::body_handle old = physics::add_body(w, {});
        physics::remove_body(w, old);
        physics::body_handle fresh = physics::add_body(w, {});
        
        CHECK(fresh.index      == old.index);
        CHECK(fresh.generation != old.generation);
        CHECK(physics::body_at(w, old)   == nullptr);
        CHECK(physics::body_at(w, fresh) != nullptr);
    }
    
    // Colliders attached to a removed body are detached too.
    {
        physics::world w;
        physics::body_handle h = physics::add_body(w, {});
        
        physics::box local { .center = {}, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
        physics::add_collider(w, h, local);
        physics::add_collider(w, h, local);
        REQUIRE(w.colliders.size() == 2u);
        
        physics::remove_body(w, h);
        
        CHECK(w.colliders.empty());
    }
}

TEST_CASE("void add_collider(world& w, body_handle attached, box local_box)")
{
    physics::world w;
    physics::body_handle h = physics::add_body(w, {});
    physics::box local { .center = { 0.5f, 0.0f }, .angle = 0.0f, .half_extents = { 1.0f, 2.0f } };
    
    physics::add_collider(w, h, local);
    
    REQUIRE(w.colliders.size() == 1u);
    CHECK(w.colliders[0].attached == h);
    check_vec2(w.colliders[0].local_box.center,       0.5f, 0.0f);
    check_vec2(w.colliders[0].local_box.half_extents, 1.0f, 2.0f);
}

TEST_CASE("rigid_body* body_at(world& w, body_handle h)")
{
    // Out-of-range index returns nullptr.
    {
        physics::world w;
        physics::add_body(w, {});
        
        CHECK(physics::body_at(w, { 42u, 1u }) == nullptr);
    }
    
    // Default-constructed handle is invalid even on an empty world.
    {
        physics::world w;
        
        CHECK(physics::body_at(w, {}) == nullptr);
    }
    
    // const overload mirrors the non-const overload.
    {
        physics::world w;
        physics::body_handle h = physics::add_body(w, {});
        
        const physics::world& cw = w;
        CHECK(physics::body_at(cw, h)        != nullptr);
        CHECK(physics::body_at(cw, { 0u, 0u }) == nullptr);
    }
}

TEST_CASE("void step(world& w, float dt)")
{
    // Integrates each live body.
    {
        physics::world w;
        physics::rigid_body state;
        state.velocity = { 2.0f, 0.0f };
        physics::body_handle h = physics::add_body(w, state);
        
        physics::step(w, 0.5f);
        
        physics::rigid_body* body = physics::body_at(w, h);
        REQUIRE(body != nullptr);
        check_vec2(body->position, 1.0f, 0.0f);
    }
    
    // Removed bodies are skipped (no crash, slot still reads as dead).
    {
        physics::world w;
        physics::rigid_body state;
        state.velocity = { 2.0f, 0.0f };
        physics::body_handle h = physics::add_body(w, state);
        physics::remove_body(w, h);
        
        physics::step(w, 1.0f);
        
        CHECK(physics::body_at(w, h) == nullptr);
    }
    
    // Emits one contact_event per overlapping collider pair.
    {
        physics::world w;
        physics::body_handle a = physics::add_body(w, {});
        physics::rigid_body close;
        close.position = { 0.5f, 0.0f };
        physics::body_handle b = physics::add_body(w, close);
        
        physics::box unit { .center = {}, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
        physics::add_collider(w, a, unit);
        physics::add_collider(w, b, unit);
        
        physics::step(w, 1.0f / 60.0f);
        
        REQUIRE(w.contact_events.size() == 1u);
        CHECK(w.contact_events[0].a == a);
        CHECK(w.contact_events[0].b == b);
    }
    
    // Clears contact events recorded before the step.
    {
        physics::world w;
        physics::body_handle a = physics::add_body(w, {});
        physics::rigid_body far_away;
        far_away.position = { 100.0f, 0.0f };
        physics::body_handle b = physics::add_body(w, far_away);
        
        physics::box unit { .center = {}, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
        physics::add_collider(w, a, unit);
        physics::add_collider(w, b, unit);
        
        w.contact_events.push_back({ a, b });
        
        physics::step(w, 1.0f / 60.0f);
        
        CHECK(w.contact_events.empty());
    }
    
    // Applies an impulse that separates closing bodies.
    {
        physics::world w;
        
        physics::rigid_body left;
        left.velocity    = { 2.0f, 0.0f };
        left.restitution = 1.0f;
        physics::body_handle a = physics::add_body(w, left);
        
        physics::rigid_body right;
        right.position    = { 1.5f, 0.0f };
        right.restitution = 1.0f;
        physics::body_handle b = physics::add_body(w, right);
        
        physics::box unit { .center = {}, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
        physics::add_collider(w, a, unit);
        physics::add_collider(w, b, unit);
        
        physics::step(w, 1.0f / 60.0f);
        
        physics::rigid_body* body_a = physics::body_at(w, a);
        physics::rigid_body* body_b = physics::body_at(w, b);
        REQUIRE(body_a != nullptr);
        REQUIRE(body_b != nullptr);
        
        CHECK(body_a->velocity.x < 2.0f);
        CHECK(body_b->velocity.x > 0.0f);
    }
    
    // Corrects positions when bodies are penetrating.
    {
        physics::world w;
        physics::body_handle a = physics::add_body(w, {});
        physics::rigid_body right;
        right.position = { 1.5f, 0.0f };
        physics::body_handle b = physics::add_body(w, right);
        
        physics::box unit { .center = {}, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
        physics::add_collider(w, a, unit);
        physics::add_collider(w, b, unit);
        
        float initial_gap = physics::body_at(w, b)->position.x
        - physics::body_at(w, a)->position.x;
        
        physics::step(w, 1.0f / 60.0f);
        
        float final_gap = physics::body_at(w, b)->position.x
        - physics::body_at(w, a)->position.x;
        
        CHECK(final_gap > initial_gap);
    }
    
    // Immovable bodies (mass = 0) absorb no impulse.
    {
        physics::world w;
        
        physics::rigid_body wall;
        wall.mass = 0.0f;
        physics::body_handle a = physics::add_body(w, wall);
        
        physics::rigid_body mover;
        mover.position    = { 1.5f, 0.0f };
        mover.velocity    = { -2.0f, 0.0f };
        mover.restitution = 1.0f;
        physics::body_handle b = physics::add_body(w, mover);
        
        physics::box unit { .center = {}, .angle = 0.0f, .half_extents = { 1.0f, 1.0f } };
        physics::add_collider(w, a, unit);
        physics::add_collider(w, b, unit);
        
        physics::step(w, 1.0f / 60.0f);
        
        physics::rigid_body* body_a = physics::body_at(w, a);
        physics::rigid_body* body_b = physics::body_at(w, b);
        REQUIRE(body_a != nullptr);
        REQUIRE(body_b != nullptr);
        
        check_vec2(body_a->velocity, 0.0f, 0.0f);
        CHECK(body_b->velocity.x > 0.0f);
    }
}
