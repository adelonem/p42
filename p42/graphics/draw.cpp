#include "draw.h"

#include "vec2.h"

namespace graphics {

void draw_oriented_rect(
    SDL_Renderer& renderer,
    math::vec2    center,
    math::vec2    size,
    float         angle,
    SDL_FColor    color
)
{
    math::vec2 f = math::forward(angle);
    math::vec2 r = math::right(angle);
    
    float half_length = size.x * 0.5f;
    float half_width  = size.y * 0.5f;
    
    math::vec2 points[4] {
        center - f * half_length - r * half_width,
        center + f * half_length - r * half_width,
        center + f * half_length + r * half_width,
        center - f * half_length + r * half_width
    };
    
    SDL_Vertex vertices[4] {};
    
    for (int i = 0; i < 4; ++i) {
        vertices[i].position  = { points[i].x, points[i].y };
        vertices[i].color     = color;
        vertices[i].tex_coord = { 0.0f, 0.0f };
    }
    
    int indices[6] { 0, 1, 2, 0, 2, 3 };
    SDL_RenderGeometry(&renderer, nullptr, vertices, 4, indices, 6);
}

} // namespace graphics
