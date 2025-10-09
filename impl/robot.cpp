/*
* frc-pathgen/impl/robot.cpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#include "robot.hpp"
#include <cmath>
#include <spdlog/spdlog.h>
#include <imgui.h>

namespace frc_pathgen {

static void draw_arc(SDL_Renderer *r, int cx, int cy, float radius,
              float start_angle, float end_angle, int segments = 64) {

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

void Robot::draw(SDL_Renderer *renderer, const Viewport &viewport) {
  float hs = this->wheelbase_m / 2.0;
  Vec2 y = this->forward();
  Vec2 x = this->right();

  Vec2 fl = this->frame_center + y*hs - x*hs;
  Vec2 fr = this->frame_center + y*hs + x*hs;
  Vec2 bl = this->frame_center - y*hs - x*hs;
  Vec2 br = this->frame_center - y*hs + x*hs;

  Vec2 flp = viewport.world_to_px(fl);
  Vec2 frp = viewport.world_to_px(fr);
  Vec2 blp = viewport.world_to_px(bl);
  Vec2 brp = viewport.world_to_px(br);

  Vec2 cp = viewport.world_to_px(this->frame_center);
  Vec2 fp = viewport.world_to_px(this->frame_center + y*hs);
  Vec2 rp = viewport.world_to_px(this->frame_center + x*hs);

  Vec2 tvp = viewport.world_to_px(this->frame_center + this->velocity_setpoint*hs / this->velocity_setpoint.length());
  Vec2 rvp = viewport.world_to_px(this->frame_center + this->velocity*hs / (this->velocity_setpoint.length() < .001?1:this->velocity_setpoint.length()));
  Vec2 pvp = viewport.world_to_px(this->frame_center + this->velocity_percent*hs);

  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

  SDL_RenderDrawLineF(renderer, flp.x, flp.y, frp.x, frp.y);
  SDL_RenderDrawLineF(renderer, frp.x, frp.y, brp.x, brp.y);
  SDL_RenderDrawLineF(renderer, brp.x, brp.y, blp.x, blp.y);
  SDL_RenderDrawLineF(renderer, blp.x, blp.y, flp.x, flp.y);

  SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
  SDL_RenderDrawLineF(renderer, cp.x, cp.y, fp.x, fp.y);
  
  SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
  SDL_RenderDrawLineF(renderer, cp.x, cp.y, rp.x, rp.y);

  SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
  SDL_RenderDrawPointF(renderer, cp.x, cp.y);


  SDL_SetRenderDrawColor(renderer, 80, 255, 255, 255);
  if (abs(this->angular_velocity_setpoint) > .001) draw_arc(renderer, cp.x, cp.y, (fp-cp).length(), -this->rotation_radians, -(this->rotation_radians + (this->angular_velocity_setpoint < 0.0 ? -1.0 : 1.0)));
  if (this->velocity_setpoint.length() > .001) SDL_RenderDrawLineF(renderer, cp.x, cp.y, tvp.x, tvp.y);

  SDL_SetRenderDrawColor(renderer, 255, 255, 80, 255);
  draw_arc(renderer, cp.x, cp.y, (fp-cp).length() * 0.975, -this->rotation_radians, -(this->rotation_radians + this->angular_velocity / abs(this->angular_velocity_setpoint)));
  if (this->velocity.length() > .001) SDL_RenderDrawLineF(renderer, cp.x, cp.y, rvp.x, rvp.y);


  SDL_SetRenderDrawColor(renderer, 255, 80, 255, 255);
  draw_arc(renderer, cp.x, cp.y, (fp-cp).length() * 0.95, -this->rotation_radians, -(this->rotation_radians + this->angular_velocity_percent));
  if (this->velocity_percent.length() > .001) SDL_RenderDrawLineF(renderer, cp.x, cp.y, pvp.x, pvp.y);

  ImGui::Begin("Robot controls");
  ImGui::Checkbox("Enable Keyboard", &this->enable_keyboard_control);
  ImGui::End();
}

void Robot::set_velocity_setpoint(Vec2 velocity) {
  this->velocity_setpoint = velocity;
}
void Robot::set_angular_velocity_setpoint(float angular_velocity) {
  this->angular_velocity_setpoint = angular_velocity;
}

void Robot::tick(float dt) {
  if (this->enable_keyboard_control) {
    const Uint8 *keys = SDL_GetKeyboardState(nullptr);
    this->velocity_setpoint.y = (keys[SDL_SCANCODE_W]?1:0) - (keys[SDL_SCANCODE_S]?1:0);
    this->velocity_setpoint.x = (keys[SDL_SCANCODE_D]?1:0) - (keys[SDL_SCANCODE_A]?1:0);

    this->angular_velocity_setpoint = (keys[SDL_SCANCODE_Q]?2:0) - (keys[SDL_SCANCODE_E]?2:0);
  }
  
  Vec2 xy_pid = this->velocity_pid.update(this->velocity_setpoint, this->velocity, dt);
  float r_pid = this->angular_velocity_pid.update(this->angular_velocity_setpoint, this->angular_velocity, dt);

  this->apply_voltages(xy_pid, r_pid, dt);

  this->frame_center += this->velocity * dt;
  this->rotation_radians += this->angular_velocity * dt;

  this->velocity *= expf(-.1 * dt);
  this->angular_velocity *= expf(-.1 * dt);
}

void Robot::apply_voltages(Vec2 xy, float r, float dt) {
  Vec2 xy_wheel_percent = xy / 12.0f;
  if (xy_wheel_percent.length() > 1.0f) {
    xy_wheel_percent *= 1.0f / xy_wheel_percent.length();
  }
  this->velocity_percent = xy_wheel_percent;

  Vec2 xy_robot_acceleration = xy_wheel_percent * this->bot_acceleration;

  float r_wheel_percent = r / 12.0f;
  if (abs(r_wheel_percent) > 1.0f) r_wheel_percent /= abs(r_wheel_percent);
  this->angular_velocity_percent = r_wheel_percent;

  float r_robot_acceleration = r_wheel_percent * this->bot_angular_acceleration;

  this->velocity += xy_robot_acceleration * dt;
  this->angular_velocity += r_robot_acceleration * dt;
}
}
