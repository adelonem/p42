#include "space_game.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

#include "draw.h"
#include "rigid_body.h"
#include "vec2.h"
#include "world.h"

namespace {

constexpr float projectile_speed { 520.0f };
constexpr float projectile_mass  { 10.0f };
constexpr float half_pi          { 1.5707963267948966f };
constexpr float two_pi           { 6.2831853071795864f };

constexpr int window_width  { 900 };
constexpr int window_height { 700 };

struct ship {
    physics::body_handle body {};
    
    float width  { 24.0f };
    float length { 48.0f };
    
    float thrust         { 900.0f };
    float reverse_thrust { 450.0f };
    float turn_torque    { 45.0f };
    
    float fire_cooldown {};
    
    float health     { 100.0f };
    float max_health { 100.0f };
};

struct enemy {
    physics::body_handle body {};
    
    float size { 80.0f };
    
    int   patrol_direction   { 1 };
    float patrol_speed       { 280.0f };
    float patrol_min_x       { 120.0f };
    float patrol_max_x       { 780.0f };
    float home_y             { 150.0f };
    float horizontal_gain    { 5.0f };
    float vertical_stiffness { 16.0f };
    float vertical_damping   { 8.0f };
    
    float fire_cooldown { 0.0f };
    float fire_interval { 0.5f };
    
    float health     { 200.0f };
    float max_health { 200.0f };
};

struct projectile {
    physics::body_handle body {};
    
    float width  { 6.0f };
    float length { 22.0f };
    
    float life { 1.4f };
};

enum class debris_palette {
    fire,
    plasma,
};

struct debris_config {
    int   particle_count    { 64 };
    float base_speed        { 180.0f };
    float speed_jitter      { 220.0f };
    float min_life          { 0.8f };
    float life_jitter       { 0.6f };
    float min_particle_size { 4.0f };
    float size_jitter       { 6.0f };
    float spin_jitter       { 8.0f };
    debris_palette palette  { debris_palette::fire };
};

struct particle {
    physics::body_handle body {};
    
    float          size     { 6.0f };
    float          life     { 0.0f };
    float          max_life { 0.0f };
    debris_palette palette  { debris_palette::fire };
};

struct debug_stats {
    float displayed_fps      {};
    float displayed_frame_ms {};
    float elapsed            {};
    int   frame_count        {};
};

} // namespace

struct space_game_state {
    physics::world simulation {};
    
    ship  player {};
    enemy target {};
    
    std::vector<projectile> projectiles {};
    std::vector<particle>   particles   {};
    
    debug_stats debug {};
};

namespace {

[[nodiscard]] physics::box box_shape(float length, float width)
{
    return {
        .center       = {},
        .angle        = 0.0f,
        .half_extents = { length * 0.5f, width * 0.5f }
    };
}

void init_game(space_game_state& game)
{
    physics::rigid_body player_body {
        .position        = { 450.0f, 600.0f },
        .damping         = 1.8f,
        .angle           = -half_pi,
        .angular_damping = 10.0f
    };
    game.player.body = physics::add_body(game.simulation, player_body);
    physics::add_collider(
        game.simulation,
        game.player.body,
        box_shape(game.player.length, game.player.width)
    );
    
    physics::rigid_body target_body {
        .position        = { 450.0f, game.target.home_y },
        .damping         = 1.5f,
        .inertia         = 100.0f,
        .angular_damping = 4.0f
    };
    game.target.body = physics::add_body(game.simulation, target_body);
    physics::add_collider(
        game.simulation,
        game.target.body,
        box_shape(game.target.size, game.target.size)
    );
}

void reset_game(space_game_state& game)
{
    game = space_game_state {};
    init_game(game);
}

void spawn_projectile(space_game_state& game)
{
    physics::rigid_body* player_body = physics::body_at(game.simulation, game.player.body);
    if (!player_body) {
        return;
    }
    
    projectile p {};
    
    math::vec2 dir = math::forward(player_body->angle);
    constexpr float spawn_margin { 2.0f };
    float spawn_distance = game.player.length * 0.5f + p.length * 0.5f + spawn_margin;
    
    physics::rigid_body initial {
        .position    = player_body->position + dir * spawn_distance,
        .velocity    = player_body->velocity + dir * projectile_speed,
        .mass        = projectile_mass,
        .angle       = player_body->angle,
        .restitution = 0.4f,
    };
    
    p.body = physics::add_body(game.simulation, initial);
    physics::add_collider(game.simulation, p.body, box_shape(p.length, p.width));
    
    game.projectiles.push_back(p);
}

void spawn_enemy_projectile(space_game_state& game)
{
    physics::rigid_body* enemy_body = physics::body_at(game.simulation, game.target.body);
    if (!enemy_body) {
        return;
    }
    
    constexpr math::vec2 down { 0.0f, 1.0f };
    
    physics::rigid_body initial {
        .position    = enemy_body->position + down * game.target.size * 0.9f,
        .velocity    = down * projectile_speed,
        .mass        = projectile_mass,
        .angle       = half_pi,
        .restitution = 0.4f,
    };
    
    projectile p {};
    p.body = physics::add_body(game.simulation, initial);
    physics::add_collider(game.simulation, p.body, box_shape(p.length, p.width));
    
    game.projectiles.push_back(p);
}

void spawn_explosion(
    space_game_state&    game,
    math::vec2           center,
    float                scale,
    const debris_config& config = {}
)
{
    static thread_local std::mt19937 rng { std::random_device{}() };
    std::uniform_real_distribution<float> unit        { 0.0f, 1.0f };
    std::uniform_real_distribution<float> signed_unit { -1.0f, 1.0f };
    
    for (int i = 0; i < config.particle_count; ++i) {
        float angle = unit(rng) * two_pi;
        float speed = config.base_speed + unit(rng) * config.speed_jitter;
        math::vec2 dir { std::cos(angle), std::sin(angle) };
        
        physics::rigid_body initial {
            .position         = center + dir * unit(rng) * scale * 0.25f,
            .velocity         = dir * speed,
            .damping          = 1.2f,
            .angle            = angle,
            .angular_velocity = signed_unit(rng) * config.spin_jitter,
            .angular_damping  = 1.5f,
        };
        
        particle p {};
        p.size     = config.min_particle_size + unit(rng) * config.size_jitter;
        p.max_life = config.min_life + unit(rng) * config.life_jitter;
        p.life     = p.max_life;
        p.palette  = config.palette;
        p.body     = physics::add_body(game.simulation, initial);
        
        game.particles.push_back(p);
    }
}

void update_player(space_game_state& game, float dt)
{
    const bool* keys = SDL_GetKeyboardState(nullptr);
    ship& player = game.player;
    
    physics::rigid_body* body = physics::body_at(game.simulation, player.body);
    if (!body) {
        return;
    }
    
    if (keys[SDL_SCANCODE_LEFT] || keys[SDL_SCANCODE_A]) {
        physics::apply_torque(*body, -player.turn_torque);
    }
    
    if (keys[SDL_SCANCODE_RIGHT] || keys[SDL_SCANCODE_D]) {
        physics::apply_torque(*body, player.turn_torque);
    }
    
    if (keys[SDL_SCANCODE_UP] || keys[SDL_SCANCODE_W]) {
        physics::apply_local_force(*body, { player.thrust, 0.0f });
    }
    
    if (keys[SDL_SCANCODE_DOWN] || keys[SDL_SCANCODE_S]) {
        physics::apply_local_force(*body, { -player.reverse_thrust, 0.0f });
    }
    
    player.fire_cooldown -= dt;
    
    if (keys[SDL_SCANCODE_SPACE] && player.fire_cooldown <= 0.0f) {
        spawn_projectile(game);
        player.fire_cooldown = 0.18f;
    }
}

void update_enemy(space_game_state& game, float dt)
{
    enemy& target = game.target;
    physics::rigid_body* body = physics::body_at(game.simulation, target.body);
    if (!body) {
        return;
    }
    
    if (body->position.x <= target.patrol_min_x) {
        target.patrol_direction = 1;
    }
    if (body->position.x >= target.patrol_max_x) {
        target.patrol_direction = -1;
    }
    
    target.fire_cooldown -= dt;
    if (target.fire_cooldown <= 0.0f) {
        spawn_enemy_projectile(game);
        target.fire_cooldown = target.fire_interval;
    }
    
    float desired_vx = target.patrol_speed * static_cast<float>(target.patrol_direction);
    float vx_accel   = (desired_vx - body->velocity.x) * target.horizontal_gain
    + desired_vx * body->damping;
    
    float y_error    = target.home_y - body->position.y;
    float vy_accel   = y_error * target.vertical_stiffness
    - body->velocity.y * target.vertical_damping;
    
    physics::apply_force(*body, { vx_accel * body->mass, vy_accel * body->mass });
}

void update_projectiles(space_game_state& game, float dt)
{
    for (projectile& p : game.projectiles) {
        p.life -= dt;
    }
    
    for (const projectile& p : game.projectiles) {
        if (p.life <= 0.0f) {
            physics::remove_body(game.simulation, p.body);
        }
    }
    
    game.projectiles.erase(
        std::remove_if(
            game.projectiles.begin(),
            game.projectiles.end(),
            [](const projectile& p) {
                return p.life <= 0.0f;
            }
        ),
        game.projectiles.end()
    );
}

void update_particles(space_game_state& game, float dt)
{
    for (particle& p : game.particles) {
        p.life -= dt;
    }
    
    for (const particle& p : game.particles) {
        if (p.life <= 0.0f) {
            physics::remove_body(game.simulation, p.body);
        }
    }
    
    game.particles.erase(
        std::remove_if(
            game.particles.begin(),
            game.particles.end(),
            [](const particle& p) {
                return p.life <= 0.0f;
            }
        ),
        game.particles.end()
    );
}

void enforce_player_boundaries(space_game_state& game)
{
    physics::rigid_body* body = physics::body_at(game.simulation, game.player.body);
    if (!body) {
        return;
    }
    
    const float c = std::abs(std::cos(body->angle));
    const float s = std::abs(std::sin(body->angle));
    const float half_extent_x = c * game.player.length * 0.5f + s * game.player.width * 0.5f;
    const float half_extent_y = s * game.player.length * 0.5f + c * game.player.width * 0.5f;
    
    const float min_x = half_extent_x;
    const float max_x = static_cast<float>(window_width)  - half_extent_x;
    const float min_y = half_extent_y;
    const float max_y = static_cast<float>(window_height) - half_extent_y;
    
    const bool out_of_bounds = body->position.x < min_x
    || body->position.x > max_x
    || body->position.y < min_y
    || body->position.y > max_y;
    
    if (out_of_bounds) {
        body->position.x = std::clamp(body->position.x, min_x, max_x);
        body->position.y = std::clamp(body->position.y, min_y, max_y);
        body->velocity         = {};
        body->angular_velocity = 0.0f;
        game.player.health     = 0.0f;
    }
}

void resolve_collisions(space_game_state& game)
{
    constexpr float contact_damage { 5.0f };
    constexpr debris_config plasma_burst {
        .particle_count    = 18,
        .base_speed        = 120.0f,
        .speed_jitter      = 160.0f,
        .min_life          = 0.25f,
        .life_jitter       = 0.4f,
        .min_particle_size = 3.0f,
        .size_jitter       = 3.0f,
        .spin_jitter       = 6.0f,
        .palette           = debris_palette::plasma,
    };
    
    for (const physics::contact_event& event : game.simulation.contact_events) {
        const bool player_hit = event.a == game.player.body || event.b == game.player.body;
        const bool enemy_hit  = event.a == game.target.body || event.b == game.target.body;
        
        if (player_hit) {
            game.player.health = std::max(0.0f, game.player.health - contact_damage * 10.f);
        }
        if (enemy_hit) {
            game.target.health = std::max(0.0f, game.target.health - contact_damage);
        }
        
        for (projectile& p : game.projectiles) {
            if (p.life <= 0.0f) {
                continue;
            }
            if (event.a != p.body && event.b != p.body) {
                continue;
            }
            
            if (const physics::rigid_body* body = physics::body_at(game.simulation, p.body)) {
                spawn_explosion(game, body->position, p.length, plasma_burst);
                physics::remove_body(game.simulation, p.body);
            }
            p.life = 0.0f;
        }
    }
    
    game.projectiles.erase(
        std::remove_if(
            game.projectiles.begin(),
            game.projectiles.end(),
            [](const projectile& p) {
                return p.life <= 0.0f;
            }
        ),
        game.projectiles.end()
    );
}

void handle_deaths(space_game_state& game)
{
    if (game.player.health <= 0.0f) {
        if (const physics::rigid_body* body = physics::body_at(game.simulation, game.player.body)) {
            spawn_explosion(game, body->position, game.player.length);
            physics::remove_body(game.simulation, game.player.body);
        }
    }
    
    if (game.target.health <= 0.0f) {
        if (const physics::rigid_body* body = physics::body_at(game.simulation, game.target.body)) {
            spawn_explosion(game, body->position, game.target.size);
            physics::remove_body(game.simulation, game.target.body);
        }
    }
}

void update_debug_stats(debug_stats& stats, float dt)
{
    stats.elapsed += dt;
    stats.frame_count += 1;
    
    if (stats.elapsed >= 1.0f) {
        stats.displayed_fps = static_cast<float>(stats.frame_count) / stats.elapsed;
        stats.displayed_frame_ms = stats.elapsed * 1000.0f / static_cast<float>(stats.frame_count);
        stats.elapsed = 0.0f;
        stats.frame_count = 0;
    }
}

[[nodiscard]] bool is_game_over(const space_game_state& game)
{
    return physics::body_at(game.simulation, game.player.body) == nullptr
    || physics::body_at(game.simulation, game.target.body) == nullptr;
}

void draw_health_bar(
    SDL_Renderer* renderer,
    math::vec2 center,
    float vertical_offset,
    float width,
    float health,
    float max_health
)
{
    constexpr float bar_height { 5.0f };
    
    float ratio = std::clamp(health / max_health, 0.0f, 1.0f);
    float left  = center.x - width * 0.5f;
    float top   = center.y - vertical_offset;
    
    SDL_FRect background { left, top, width, bar_height };
    SDL_SetRenderDrawColor(renderer, 60, 60, 70, 255);
    SDL_RenderFillRect(renderer, &background);
    
    SDL_FRect foreground { left, top, width * ratio, bar_height };
    SDL_SetRenderDrawColor(renderer, 100, 220, 80, 255);
    SDL_RenderFillRect(renderer, &foreground);
}

[[nodiscard]] int rendered_object_count(const space_game_state& game)
{
    int count = 0;
    
    if (physics::body_at(game.simulation, game.player.body)) {
        ++count;
    }
    if (physics::body_at(game.simulation, game.target.body)) {
        ++count;
    }
    
    for (const projectile& p : game.projectiles) {
        if (physics::body_at(game.simulation, p.body)) {
            ++count;
        }
    }
    
    for (const particle& p : game.particles) {
        if (physics::body_at(game.simulation, p.body)) {
            ++count;
        }
    }
    
    return count;
}

void draw_debug_text(SDL_Renderer* renderer, const space_game_state& game)
{
    constexpr float line_height { 10.0f };
    char line[128] {};
    
    SDL_SetRenderDrawColor(renderer, 210, 230, 255, 255);
    
    std::snprintf(
        line,
        sizeof(line),
        "FPS: %.0f  frame: %.2f ms",
        game.debug.displayed_fps,
        game.debug.displayed_frame_ms
    );
    SDL_RenderDebugText(renderer, 8.0f, 8.0f, line);
    
    std::snprintf(line, sizeof(line), "Rendered objects: %d", rendered_object_count(game));
    SDL_RenderDebugText(renderer, 8.0f, 18.0f, line);
    
    std::snprintf(
        line,
        sizeof(line),
        "Bodies: %zu  colliders: %zu  contacts: %zu",
        game.simulation.bodies.size(),
        game.simulation.colliders.size(),
        game.simulation.contact_events.size()
    );
    SDL_RenderDebugText(renderer, 8.0f, 8.0f + line_height * 2.0f, line);
    
    std::snprintf(
        line,
        sizeof(line),
        "Projectiles: %zu  particles: %zu",
        game.projectiles.size(),
        game.particles.size()
    );
    SDL_RenderDebugText(renderer, 8.0f, 8.0f + line_height * 3.0f, line);
}

void draw_centered_debug_text(SDL_Renderer* renderer, float y, const char* text)
{
    float text_width = static_cast<float>(std::strlen(text) * SDL_DEBUG_TEXT_FONT_CHARACTER_SIZE);
    float x = (static_cast<float>(window_width) - text_width) * 0.5f;
    SDL_RenderDebugText(renderer, x, y, text);
}

void draw_game_over_text(SDL_Renderer* renderer)
{
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    draw_centered_debug_text(renderer, 320.0f, "Game over");
    draw_centered_debug_text(renderer, 340.0f, "Press Enter to restart");
}

void render_world(SDL_Renderer* renderer, const space_game_state& game)
{
    SDL_SetRenderDrawColor(renderer, 10, 10, 16, 255);
    SDL_RenderClear(renderer);
    
    constexpr SDL_FColor white  { 1.0f, 1.0f,  1.0f, 1.0f };
    constexpr SDL_FColor orange { 1.0f, 0.55f, 0.1f, 1.0f };
    
    if (const physics::rigid_body* body = physics::body_at(game.simulation, game.player.body)) {
        graphics::draw_oriented_rect(
            *renderer,
            body->position,
            { game.player.length, game.player.width },
            body->angle,
            white
        );
        draw_health_bar(
            renderer,
            body->position,
            game.player.length * 0.5f + 14.0f,
            game.player.length,
            game.player.health,
            game.player.max_health
        );
    }
    
    if (const physics::rigid_body* body = physics::body_at(game.simulation, game.target.body)) {
        graphics::draw_oriented_rect(
            *renderer,
            body->position,
            { game.target.size, game.target.size },
            body->angle,
            orange
        );
        draw_health_bar(
            renderer,
            body->position,
            game.target.size * 0.5f + 14.0f,
            game.target.size,
            game.target.health,
            game.target.max_health
        );
    }
    
    for (const projectile& p : game.projectiles) {
        if (const physics::rigid_body* body = physics::body_at(game.simulation, p.body)) {
            graphics::draw_oriented_rect(*renderer, body->position, { p.length, p.width }, body->angle, white);
        }
    }
    
    for (const particle& p : game.particles) {
        if (const physics::rigid_body* body = physics::body_at(game.simulation, p.body)) {
            float t = 1.0f - p.life / p.max_life;
            SDL_FColor color { 1.0f, 1.0f, 1.0f, 1.0f };
            
            switch (p.palette) {
                case debris_palette::fire: {
                    float g = std::clamp((1.0f - t) * 1.5f, 0.0f, 1.0f);
                    float b = std::clamp(1.0f - 3.0f * t,   0.0f, 1.0f);
                    color = { 1.0f, g, b, 1.0f };
                    break;
                }
                case debris_palette::plasma:
                    color = {
                        1.0f - t * 0.5f,
                        1.0f - t * 0.2f,
                        1.0f,
                        1.0f
                    };
                    break;
            }
            
            graphics::draw_oriented_rect(*renderer, body->position, { p.size, p.size }, body->angle, color);
        }
    }
    
    if (is_game_over(game)) {
        draw_game_over_text(renderer);
    }
    
    draw_debug_text(renderer, game);
    SDL_RenderPresent(renderer);
}

} // namespace

space_game::space_game()
: state { std::make_unique<space_game_state>() }
{
    init_game(*state);
}

space_game::~space_game() = default;

void space_game::handle_event(const SDL_Event& event, game_controls& /*controls*/)
{
    if (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_RETURN) {
        reset_game(*state);
    }
}

void space_game::update(float dt, game_controls& /*controls*/)
{
    update_debug_stats(state->debug, dt);
}

void space_game::fixed_update(float dt, game_controls& /*controls*/)
{
    update_player(*state, dt);
    update_enemy(*state, dt);
    update_projectiles(*state, dt);
    update_particles(*state, dt);
    physics::step(state->simulation, dt);
    resolve_collisions(*state);
    enforce_player_boundaries(*state);
    handle_deaths(*state);
}

void space_game::render(SDL_Renderer& renderer)
{
    render_world(&renderer, *state);
}
