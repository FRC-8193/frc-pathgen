/*
* frc-pathgen/include/path.hpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#pragma once

#include "vec2.hpp"
#include "viewport.hpp"
#include <SDL2/SDL.h>

namespace frc_pathgen {

class Path {
public:
  virtual Vec2 sample_position(float t) const = 0;
  // max(||d^2/dt^2 position(t)||)
  virtual float max_acceleration() const = 0;

  virtual ~Path() = 0;
};

inline Path::~Path() = default;

// smoothstepped lerp
class LinePath : public Path {
public:
  LinePath(Vec2 a, Vec2 b);

  virtual Vec2 sample_position(float t) const override;
  virtual float max_acceleration() const override;

  void draw(SDL_Renderer *renderer, Viewport &viewport);
  bool consume_event(SDL_Event &e);

  virtual ~LinePath() override = default;
private:
  Vec2 a, b;
};
}
