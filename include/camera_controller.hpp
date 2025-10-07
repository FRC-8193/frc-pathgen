/*
* frc-pathgen/include/camera_controller.hpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#pragma once

#include <SDL2/SDL.h>
#include "viewport.hpp"

namespace frc_pathgen {

class CameraController {
public:
  CameraController(Viewport &viewport) : viewport(viewport) {
  }

  bool consume_event(SDL_Event &e);
private:
  Viewport &viewport;
  bool mouse_down = false;
  Vec2 last_mouse_pos;
};
}
