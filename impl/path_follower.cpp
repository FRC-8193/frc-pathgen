/*
* frc-pathgen/impl/path_follower.cpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#include "path_follower.hpp"
#include "gfx.hpp"
#include <spdlog/spdlog.h>
#include <imgui.h>
#include <algorithm>

namespace frc_pathgen {

PathFollower::PathFollower(Robot &robot) : robot(robot), 
  position_pid(40.0, 0.0, 2.0), angle_pid(7.5, 0.0, 1.5) {
}

void PathFollower::set_path(Path &path) {
  this->path = &path;
}

void PathFollower::draw(SDL_Renderer *renderer, const Viewport &viewport) {
  Vec2 tp = viewport.world_to_px(this->target);
  Vec2 gp = viewport.world_to_px(this->target+this->gradient);

  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  draw_filled_circle(renderer, tp.x, tp.y, 10);
  SDL_RenderDrawLineF(renderer, tp.x, tp.y, gp.x, gp.y);

  ImGui::Begin("Path Following Controls");
  ImGui::SliderFloat("Velocity Feedforward", &this->feedforward, 0.0f, 1.0f);
  ImGui::End();
}

void PathFollower::tick(float dt) {
  static const float EPS = .001;

  float t = this->time;

  if (t > 1.0) this->time = 0.0;

  Vec2 path_last    = this->path? this->path->sample_position(t-EPS) : Vec2 { 0,0 };
  Vec2 path_current = this->path? this->path->sample_position(t) : Vec2 { 0,0 };
  Vec2 path_next    = this->path? this->path->sample_position(t+EPS) : Vec2 { 0,0 };

  Vec2 gradient = (path_next-path_last) / (EPS * 2.0);
  
  Vec2 accel = (path_next - 2.0f * path_current + path_last) / (EPS * EPS);

  float timescale = sqrtf(Robot::bot_acceleration / accel.length());
  timescale /= (1.0 + (this->robot.get_frame_center()-path_current).length());
  timescale /= (1.0 + (this->robot.get_velocity()-this->timescale*gradient).length());
  this->gradient = gradient*timescale;
  this->timescale = timescale;

  Vec2 position_setpoint = path_current;
  this->target = position_setpoint;
  float angle_setpoint = 0.0;

  this->time += dt * timescale;

  Vec2 pos = this->robot.get_frame_center();
  float angle = this->robot.get_rotation_radians();

  this->position_pid.kD = 10.0 * this->robot.get_velocity().length();

  Vec2 velocity_setpoint = this->position_pid.update(position_setpoint, pos, dt) + this->feedforward * gradient * timescale;
  float angular_velocity_setpoint = this->angle_pid.update(angle_setpoint, angle, dt);

  this->robot.set_velocity_setpoint(velocity_setpoint);
  this->robot.set_angular_velocity_setpoint(angular_velocity_setpoint);
}
/*
// newton-rhapson
float PathFollower::find_nearest_t() {
  if (!path) return 0.0f;

  const int MAX_ITER = 5;
  const float EPSILON = 1e-4f;
  const float DT = 0.01f; // small delta for finite difference

  float t = this->last_path_t;        // initial guess
  Vec2 R = this->robot.get_frame_center();    // robot current position

  for (int i = 0; i < MAX_ITER; ++i) {
    // finite differences for first derivative
    float t_forward = std::min(t + DT, 1.0f);
    float t_backward = std::max(t - DT, 0.0f);

    Vec2 B_forward = this->path->sample_position(t_forward);
    Vec2 B_backward = this->path->sample_position(t_backward);

    Vec2 dB = (B_forward - B_backward) * (0.5f / DT);

    // finite differences for second derivative
    Vec2 B_current = this->path->sample_position(t);
    Vec2 ddB = (B_forward - B_current * 2.0f + B_backward) * (1.0f / (DT * DT));

    Vec2 diff = B_current - R;

    float f_prime = 2.0f * (diff.x * dB.x + diff.y * dB.y);
    float f_double_prime = 2.0f * (dB.x*dB.x + dB.y*dB.y + diff.x*ddB.x + diff.y*ddB.y);

    if (std::abs(f_double_prime) < 1e-6f)
      break;

    float t_next = t - f_prime / f_double_prime;
    t_next = std::clamp(t_next, 0.0f, 1.0f);

    if (fabsf(t_next - t) < EPSILON) {
      break;
    }

    t = t_next;
  }

  this->last_path_t = t;
  return t;
}*/

}
