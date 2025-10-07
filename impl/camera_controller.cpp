/*
* frc-pathgen/impl/camera_controller.cpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#include "camera_controller.hpp"

namespace frc_pathgen {

bool CameraController::consume_event(SDL_Event &e) {
  switch (e.type) {
  case SDL_MOUSEBUTTONDOWN:
    if (e.button.button == SDL_BUTTON_LEFT) {
      this->mouse_down = true;
      this->last_mouse_pos.x = e.button.x;
      this->last_mouse_pos.y = e.button.y;
    }
    return true;
  case SDL_MOUSEBUTTONUP:
    if (e.button.button == SDL_BUTTON_LEFT) {
      this->mouse_down = false;
    }
    return true;
  case SDL_MOUSEMOTION: {
    if (!this->mouse_down) return false;
    int dx = e.button.x - this->last_mouse_pos.x;
    int dy = e.button.y - this->last_mouse_pos.y;

    this->last_mouse_pos.x = e.button.x;
    this->last_mouse_pos.y = e.button.y;

    float units_per_px = this->viewport.units_per_vw / this->viewport.width;

    this->viewport.center.x -= dx * units_per_px; 
    this->viewport.center.y += dy * units_per_px;
    return true; }
  case SDL_MOUSEWHEEL: {
    if (e.wheel.y == 0) return false;
    float fac = powf(1.1, e.wheel.y);
    int mx, my;
    SDL_GetMouseState(&mx, &my);
    
    Vec2 mouse = { (float)mx, (float)my };
    Vec2 pre_world = this->viewport.px_to_world(mouse);

    this->viewport.units_per_vw *= fac;

    Vec2 post_world = this->viewport.px_to_world(mouse);

    this->viewport.center += pre_world - post_world;

    return true; }
  default:
    return false;
  }
}
}
