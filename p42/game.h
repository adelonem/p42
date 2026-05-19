#pragma once

#include <SDL3/SDL.h>

// Channel through which the game asks the host application to quit.
// The application owns the instance and consults it after each callback.
struct game_controls {
    bool quit_requested {};
};

class game {
public:
    virtual ~game() = default;

    virtual void handle_event(const SDL_Event& event, game_controls& controls) = 0;
    virtual void update(float dt, game_controls& controls)                     = 0;
    virtual void fixed_update(float dt, game_controls& controls)               = 0;
    virtual void render(SDL_Renderer& renderer)                                = 0;
};
