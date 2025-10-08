/*
* frc-pathgen/include/world.hpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#pragma once

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include "viewport.hpp"

namespace frc_pathgen {

void draw_world_gridlines(SDL_Renderer *renderer, TTF_Font *font, const Viewport &viewport);
}
