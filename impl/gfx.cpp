/*
* frc-pathgen/impl/gfx.cpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#include "gfx.hpp"

namespace frc_pathgen {

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

void draw_arc(SDL_Renderer *r, int cx, int cy, float radius,
              float start_angle, float end_angle, int segments) {

  if (abs(start_angle - end_angle) < .01) return;

  float step = (end_angle - start_angle) / segments;
  for (int i = 0; i < segments; i++) {
    float a0 = start_angle + i * step;
    float a1 = a0 + step;

    int x0 = cx + (int)(cosf(a0) * radius);
    int y0 = cy + (int)(sinf(a0) * radius);
    int x1 = cx + (int)(cosf(a1) * radius);
    int y1 = cy + (int)(sinf(a1) * radius);

    SDL_RenderDrawLine(r, x0, y0, x1, y1);
  }
}

void draw_filled_circle(SDL_Renderer *r, int cx, int cy, int radius) {
  for (int dy = -radius; dy <= radius; dy++) {
    int dx = (int)std::sqrt(radius * radius - dy * dy);
    SDL_RenderDrawLine(r, cx - dx, cy + dy, cx + dx, cy + dy);
  }
}
}
