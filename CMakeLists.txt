cmake_minimum_required(VERSION 3.25)
project(p42 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Linux:  sudo apt install libsdl3-dev
# macOS:  brew install sdl3
find_package(SDL3 REQUIRED CONFIG)

# Pure C++20 math + physics — no SDL3 dependency.
# PUBLIC include dirs propagate "vec2.h", "rigid_body.h", etc.
# to every consumer so that bare includes keep working unchanged.
add_library(p42_core STATIC
    p42/math/vec2.cpp
    p42/physics/rigid_body.cpp
    p42/physics/collision.cpp
    p42/physics/world.cpp
)

target_include_directories(p42_core PUBLIC
    ${CMAKE_SOURCE_DIR}/p42/math
    ${CMAKE_SOURCE_DIR}/p42/physics
)

target_compile_features(p42_core PUBLIC cxx_std_20)

# Game executable — links SDL3 and p42_core.
add_executable(p42
    p42/main.cpp
    p42/application.cpp
    p42/space_game.cpp
    p42/graphics/draw.cpp
)

target_include_directories(p42 PRIVATE
    ${CMAKE_SOURCE_DIR}/p42
    ${CMAKE_SOURCE_DIR}/p42/graphics
)

target_link_libraries(p42 PRIVATE
    p42_core
    SDL3::SDL3
)

# Doctest test runner — no SDL3 dependency.
add_executable(p42_tests
    p42_tests/main.cpp
    p42_tests/vec2_tests.cpp
    p42_tests/collision_tests.cpp
    p42_tests/rigid_body_tests.cpp
    p42_tests/world_tests.cpp
)

target_include_directories(p42_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/p42_tests
)

target_link_libraries(p42_tests PRIVATE
    p42_core
)
