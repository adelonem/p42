#include "collision.h"

#include <algorithm>
#include <cmath>

namespace physics {

namespace {

constexpr float epsilon { 0.0001f };

float radius_on(
    const box& shape,
    math::vec2 box_x,
    math::vec2 box_y,
    math::vec2 axis
)
{
    return std::abs(math::dot(box_x, axis)) * shape.half_extents.x
         + std::abs(math::dot(box_y, axis)) * shape.half_extents.y;
}

void vertices_of(const box& shape, math::vec2 vertices[4])
{
    math::vec2 x = math::forward(shape.angle) * shape.half_extents.x;
    math::vec2 y = math::right(shape.angle) * shape.half_extents.y;
    
    vertices[0] = shape.center - x - y;
    vertices[1] = shape.center + x - y;
    vertices[2] = shape.center + x + y;
    vertices[3] = shape.center - x + y;
}

bool contains(const box& shape, math::vec2 point)
{
    math::vec2 local = math::rotate(point - shape.center, -shape.angle);
    
    return std::abs(local.x) <= shape.half_extents.x + epsilon
        && std::abs(local.y) <= shape.half_extents.y + epsilon;
}

bool segment_intersection(
    math::vec2 a0,
    math::vec2 a1,
    math::vec2 b0,
    math::vec2 b1,
    math::vec2& intersection
)
{
    math::vec2 r = a1 - a0;
    math::vec2 s = b1 - b0;
    float denominator = math::cross(r, s);
    
    if (std::abs(denominator) <= epsilon) {
        return false;
    }
    
    math::vec2 delta = b0 - a0;
    float t = math::cross(delta, s) / denominator;
    float u = math::cross(delta, r) / denominator;
    
    if (t < -epsilon || t > 1.0f + epsilon || u < -epsilon || u > 1.0f + epsilon) {
        return false;
    }
    
    intersection = a0 + r * std::clamp(t, 0.0f, 1.0f);
    return true;
}

void add_contact(math::vec2 contacts[16], int& count, math::vec2 point)
{
    for (int i = 0; i < count; ++i) {
        if (math::length_squared(contacts[i] - point) <= epsilon * epsilon) {
            return;
        }
    }
    
    contacts[count] = point;
    ++count;
}

math::vec2 average_contacts(const box& a, const box& b)
{
    math::vec2 a_vertices[4] {};
    math::vec2 b_vertices[4] {};
    vertices_of(a, a_vertices);
    vertices_of(b, b_vertices);
    
    math::vec2 contacts[16] {};
    int contact_count = 0;
    
    for (math::vec2 vertex : a_vertices) {
        if (contains(b, vertex)) {
            add_contact(contacts, contact_count, vertex);
        }
    }
    
    for (math::vec2 vertex : b_vertices) {
        if (contains(a, vertex)) {
            add_contact(contacts, contact_count, vertex);
        }
    }
    
    for (int i = 0; i < 4; ++i) {
        math::vec2 a0 = a_vertices[i];
        math::vec2 a1 = a_vertices[(i + 1) % 4];
        
        for (int j = 0; j < 4; ++j) {
            math::vec2 b0 = b_vertices[j];
            math::vec2 b1 = b_vertices[(j + 1) % 4];
            math::vec2 intersection {};
            
            if (segment_intersection(a0, a1, b0, b1, intersection)) {
                add_contact(contacts, contact_count, intersection);
            }
        }
    }
    
    if (contact_count == 0) {
        return (a.center + b.center) * 0.5f;
    }
    
    math::vec2 contact {};
    
    for (int i = 0; i < contact_count; ++i) {
        contact += contacts[i];
    }
    
    return contact / static_cast<float>(contact_count);
}

} // namespace

std::optional<collision> intersect(const box& a, const box& b)
{
    math::vec2 a_x = math::forward(a.angle);
    math::vec2 a_y = math::right(a.angle);
    math::vec2 b_x = math::forward(b.angle);
    math::vec2 b_y = math::right(b.angle);
    
    math::vec2 axes[4] { a_x, a_y, b_x, b_y };
    math::vec2 delta = a.center - b.center;
    
    float min_overlap = std::numeric_limits<float>::infinity();
    math::vec2 mtv_axis {};
    
    for (math::vec2 axis : axes) {
        float ra = radius_on(a, a_x, a_y, axis);
        float rb = radius_on(b, b_x, b_y, axis);
        float distance = std::abs(math::dot(delta, axis));
        float overlap = ra + rb - distance;
        
        if (overlap <= 0.0f) {
            return std::nullopt;
        }
        
        if (overlap < min_overlap) {
            min_overlap = overlap;
            mtv_axis = math::dot(delta, axis) < 0.0f ? -axis : axis;
        }
    }
    
    return collision {
        .contact = average_contacts(a, b),
        .normal  = mtv_axis,
        .depth   = min_overlap
    };
}

} // namespace physics
