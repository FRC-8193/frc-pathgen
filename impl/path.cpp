/*
* frc-pathgen/impl/path.cpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#include "path.hpp"

namespace frc_pathgen {

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

Vec2 BezierPath::sample_position(float t) const {
  return powf(1.0f-t, 3.0f)*this->p0 + 3.0f*powf(1.0f-t, 2.0f)*t*this->p1 + 3.0f*(1.0f-t)*t*t*this->p2 + t*t*t*this->p3;
}

float BezierPath::max_acceleration() const {
  constexpr int STEPS = 256;
  float max_accel = 0.0f;

  for (int i = 0; i <= STEPS; ++i) {
    float t = (float)i / (float)STEPS;

    Vec2 a = this->p2 - this->p1 * 2.0f + this->p0;
    Vec2 b = this->p3 - this->p2 * 2.0f + this->p1;
    Vec2 accel = (b - a) * (6.0f * t) + (a * 6.0f);

    float accel_mag = sqrtf(accel.x * accel.x + accel.y * accel.y);
    if (accel_mag > max_accel)
      max_accel = accel_mag;
  }

  return max_accel;
}

void BezierPath::draw(SDL_Renderer *renderer, Viewport &viewport) {
  constexpr int STEPS = 256;

  Vec2 last = this->p0;

  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

  for (int i = 0; i <= STEPS; ++i) {
    float t = (float)i / (float)STEPS;

    Vec2 p = this->sample_position(t);

    Vec2 lp = viewport.world_to_px(last);
    Vec2 pp = viewport.world_to_px(p);

    last = p;

    SDL_RenderDrawLineF(renderer, lp.x, lp.y, pp.x, pp.y);
  }
}

bool BezierPath::consume_event(SDL_Event &e) {
  return false;
}
}
