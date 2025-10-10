/*
* frc-pathgen/impl/path.cpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#include "path.hpp"

namespace frc_pathgen {

LinePath::LinePath(Vec2 a, Vec2 b) : a(a), b(b) {
}

Vec2 LinePath::sample_position(float t) const {
  return (this->b - this->a) * (3.0f*t*t - 2.0f*t*t*t) + this->a;
}

float LinePath::max_acceleration() const {
  return 6.0f * (this->b - this->a).length();
}

void LinePath::draw(SDL_Renderer *renderer, Viewport &viewport) {
  Vec2 ap = viewport.world_to_px(this->a);
  Vec2 bp = viewport.world_to_px(this->b);

  SDL_SetRenderDrawColor(renderer, 128, 128, 128, 255);
  SDL_RenderDrawLineF(renderer, ap.x, ap.y, bp.x, bp.y);
}

bool LinePath::consume_event(SDL_Event &e) {
  return false;
}
}
