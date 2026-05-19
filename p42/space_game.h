#pragma once

#include <memory>

#include "game.h"

struct space_game_state;

class space_game final : public game {
public:
    space_game();
    ~space_game() override;
    
    space_game(const space_game&) = delete;
    space_game& operator=(const space_game&) = delete;
    
    void handle_event(const SDL_Event& event, game_controls& controls) override;
    void update(float dt, game_controls& controls) override;
    void fixed_update(float dt, game_controls& controls) override;
    void render(SDL_Renderer& renderer) override;
    
private:
    std::unique_ptr<space_game_state> state;
};
