/*
* frc-pathgen/include/path_follower.hpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#pragma once

#include "pid.hpp"
#include "viewport.hpp"
#include "vec2.hpp"
#include "robot.hpp"
#include "path.hpp"
#include <SDL2/SDL.h>

namespace frc_pathgen {

class PathFollower {
public:
  PathFollower(Robot &robot);
  
  void set_path(Path &path);

  void draw(SDL_Renderer *renderer, const Viewport &viewport);

  void tick(float dt);
private:
  float time = 0.0f;
  float duration = 10.0f;
  float feedforward = 0.6f;
  Path *path = nullptr;
  Robot &robot;
  PIDController<Vec2, float> position_pid;
  Vec2 target;
  PIDController<float> angle_pid;
};
}
