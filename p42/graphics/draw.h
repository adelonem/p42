#pragma once

#include <SDL3/SDL.h>

#include "vec2.h"

namespace graphics {

void draw_oriented_rect(
    SDL_Renderer& renderer,
    math::vec2    center,
    math::vec2    size,
    float         angle,
    SDL_FColor    color
);

} // namespace graphics
