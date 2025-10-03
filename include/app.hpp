/*
* frc-pathgen/include/app.hpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#pragma once

#include <SDL2/SDL.h>
#include "viewport.hpp"

namespace frc_pathgen {

class App {
public:
  App();

  inline bool is_ok() const { return this->window != nullptr; }

  void run();
private:
  bool process_event(const SDL_Event &e);
  void teardown();

  SDL_Window *window;
  SDL_Renderer *renderer;
  Viewport viewport;
};
}
