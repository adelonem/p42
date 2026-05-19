#include "application.h"

int run_application(const application_configuration& configuration, game& game)
{
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return 1;
    }
    
    SDL_Window*   window   = nullptr;
    SDL_Renderer* renderer = nullptr;
    if (!SDL_CreateWindowAndRenderer(configuration.title, configuration.width, configuration.height, 0, &window, &renderer)) {
        SDL_Log("SDL_CreateWindowAndRenderer failed: %s", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    
    constexpr float fixed_dt     { 1.0f / 60.0f };
    constexpr float max_frame_dt { 0.25f };
    
    game_controls controls {};
    SDL_Event event        {};
    bool          running        = true;
    Uint64        previous_ticks = SDL_GetTicks();
    float         accumulator    = 0.0f;
    
    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT) {
                running = false;
            }
            else {
                game.handle_event(event, controls);
            }
        }
        
        Uint64 current_ticks = SDL_GetTicks();
        float frame_dt = static_cast<float>(current_ticks - previous_ticks) / 1000.0f;
        previous_ticks = current_ticks;
        
        if (frame_dt > max_frame_dt) {
            frame_dt = max_frame_dt;
        }
        
        accumulator += frame_dt;
        
        while (accumulator >= fixed_dt) {
            game.fixed_update(fixed_dt, controls);
            accumulator -= fixed_dt;
            
            if (controls.quit_requested) {
                break;
            }
        }
        
        game.update(frame_dt, controls);
        game.render(*renderer);
        
        if (controls.quit_requested) {
            running = false;
        }
    }
    
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
