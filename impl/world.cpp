/*
* frc-pathgen/impl/world.cpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#include "world.hpp"
#include <cmath>
#include <string>

namespace frc_pathgen {

// Simple helper for text rendering
void draw_text(SDL_Renderer *r, TTF_Font *font, const std::string &text, float x, float y) {
  if (!font) return;
  SDL_Color color = {200, 200, 200, 255};
  SDL_Surface *surf = TTF_RenderText_Blended(font, text.c_str(), color);
  SDL_Texture *tex = SDL_CreateTextureFromSurface(r, surf);
  SDL_Rect dst = {(int)x, (int)y, surf->w, surf->h};
  SDL_RenderCopy(r, tex, nullptr, &dst);
  SDL_FreeSurface(surf);
  SDL_DestroyTexture(tex);
}

void draw_world_gridlines(SDL_Renderer *r, TTF_Font *font, const Viewport &vp) {
  float units_per_px = vp.units_per_vw / vp.width;
  float px_per_unit  = 1.0f / units_per_px;

  float target_px_spacing = 80.0f;
  float raw_spacing = target_px_spacing * units_per_px;
  float pow10 = powf(10.0f, floorf(log10f(raw_spacing)));
  float norm = raw_spacing / pow10;
  float nice_factor = (norm < 2 ? 1 : (norm < 10 ? 2 : 10));
  float grid_spacing = nice_factor * pow10;

  float left   = vp.center.x - vp.units_per_vw / 2.0f;
  float right  = vp.center.x + vp.units_per_vw / 2.0f;
  float height_units = vp.units_per_vw * (vp.height / (float)vp.width);
  float top    = vp.center.y + height_units / 2.0f;
  float bottom = vp.center.y - height_units / 2.0f;

  float start_x = floorf(left / grid_spacing) * grid_spacing;
  float start_y = floorf(bottom / grid_spacing) * grid_spacing;

  int major_every = 5;

  // --- Vertical lines ---
  for (int i = 0; ; ++i) {
    float x = start_x + i * grid_spacing;
    if (x > right) break;

    bool is_major = ((int)roundf(x / grid_spacing)) % major_every == 0;

    if (fabsf(x) < 1e-5f) {
      SDL_SetRenderDrawColor(r, 80, 255, 80, 255); // Y axis
    } else if (is_major) {
      SDL_SetRenderDrawColor(r, 120, 120, 120, 255);
    } else {
      SDL_SetRenderDrawColor(r, 80, 80, 80, 120);
    }

    Vec2 p0 = vp.world_to_px({x, bottom});
    Vec2 p1 = vp.world_to_px({x, top});
    SDL_RenderDrawLineF(r, p0.x, p0.y, p1.x, p1.y);

    // --- Label ---
    if (is_major && font && fabsf(x) > 1e-5f) {
      Vec2 label_pos = vp.world_to_px({x, 0});
      draw_text(r, font, std::to_string((int)x), label_pos.x + 2, label_pos.y + 2);
    }
  }

  // --- Horizontal lines ---
  for (int j = 0; ; ++j) {
    float y = start_y + j * grid_spacing;
    if (y > top) break;

    bool is_major = ((int)roundf(y / grid_spacing)) % major_every == 0;

    if (fabsf(y) < 1e-5f) {
      SDL_SetRenderDrawColor(r, 255, 80, 80, 255); // X axis
    } else if (is_major) {
      SDL_SetRenderDrawColor(r, 120, 120, 120, 255);
    } else {
      SDL_SetRenderDrawColor(r, 80, 80, 80, 120);
    }

    Vec2 p0 = vp.world_to_px({left, y});
    Vec2 p1 = vp.world_to_px({right, y});
    SDL_RenderDrawLineF(r, p0.x, p0.y, p1.x, p1.y);

    // --- Label ---
    if (is_major && font && fabsf(y) > 1e-5f) {
      Vec2 label_pos = vp.world_to_px({0, y});
      draw_text(r, font, std::to_string((int)y), label_pos.x + 4, label_pos.y + 4);
    }
  }
}

}
