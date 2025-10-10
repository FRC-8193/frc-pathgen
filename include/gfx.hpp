/*
* frc-pathgen/include/gfx.hpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#pragma once

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <string>

namespace frc_pathgen {
void draw_text(SDL_Renderer *r, TTF_Font *font, const std::string &text, float x, float y);
void draw_arc(SDL_Renderer *r, int cx, int cy, float radius,
              float start_angle, float end_angle, int segments = 64);
}
