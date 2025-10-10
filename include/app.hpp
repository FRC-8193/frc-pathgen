/*
* frc-pathgen/include/app.hpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#pragma once

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include "camera_controller.hpp"
#include "robot.hpp"
#include "path_follower.hpp"

namespace frc_pathgen {

class App {
public:
  App();

  inline bool is_ok() const { return this->window != nullptr; }
  
  void run();
private:
  void teardown();

  SDL_Window *window;
  SDL_Renderer *renderer;
  TTF_Font *grid_font;
  TTF_Font *fps_font;
  Viewport viewport;

  Robot robot;
  CameraController camera_controller;
  PathFollower path_follower;
};
}
