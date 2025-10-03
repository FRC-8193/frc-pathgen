/*
* frc-pathgen/include/camera.hpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#include "vec2.hpp"

namespace frc_pathgen {

struct Viewport {
  Vec2 center = {0,0}; // X/Y coordinates of the *center* of the screen (world units)
  float units_per_vw = 1; // # of world units per viewport width
  
  unsigned int width, height; // Size of the viewport (pixels)

  inline Vec2 world_to_ndc(Vec2 world) const {
    Vec2 c = (world - this->center) / (this->units_per_vw * 2.0f);
    return {c.x, -c.y};
  }

  inline Vec2 ndc_to_px(Vec2 ndc) const {
    return Vec2{
      (ndc.x * 0.5f + 0.5f) * this->width,
      (ndc.y * 0.5f + 0.5f) * this->width + this->height/2.0f - this->width/2.0f
    };
  }

  inline Vec2 world_to_px(Vec2 world) const {
    return ndc_to_px(world_to_ndc(world));
  }
};
}
