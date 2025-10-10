/*
* frc-pathgen/include/vec2.hpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#pragma once

#include <cmath>

namespace frc_pathgen {

struct Vec2 {
  float x, y;

  inline float length() const { return sqrtf(x*x+y*y); }

  inline Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
  inline Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
  inline Vec2 operator*(float f) const { return {x * f, y * f}; }
  inline Vec2 operator/(float f) const { return {x / f, y / f}; }
  inline Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
  inline Vec2& operator-=(const Vec2& o) { x -= o.x; y -= o.y; return *this; }
  inline Vec2& operator*=(const float& o) { x *= o; y *= o; return *this; }

  inline static float dot(const Vec2& a, const Vec2& b) { return a.x*b.x+a.y*b.y; }
};

inline Vec2 operator*(float a, const Vec2& b) {
  return b*a;
}
}
