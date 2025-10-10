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
  position_pid(40.0, 0.0, 2.0), angle_pid(7.5, 0.0, 1.5), feedforward(0.4) {
}

void PathFollower::set_path(Path &path) {
  this->duration = sqrtf(path.max_acceleration() / Robot::bot_acceleration) * 1.1; // Keep the robot running as hard as possible

  this->path = &path;

  spdlog::info("Calculated duration {}", this->duration);
}

void PathFollower::draw(SDL_Renderer *renderer, const Viewport &viewport) {
  Vec2 tp = viewport.world_to_px(this->target);
  Vec2 fp = viewport.world_to_px(this->path->sample_position(this->last_path_t+this->lookahead*this->robot.get_velocity().length()));

  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  draw_filled_circle(renderer, tp.x, tp.y, 10);
  draw_filled_circle(renderer, fp.x, fp.y, 10);

  ImGui::Begin("Path Follower Controls");
  ImGui::SliderFloat("Velocity feedforward", &this->feedforward, 0.0f, 1.0f);
  ImGui::SliderFloat("Turn angle lookahead", &this->lookahead, 0.0f, 1.0f);
  ImGui::End();
}

void PathFollower::tick(float dt) {
  static const float EPS = .001;

  float t = this->find_nearest_t();

  if (t > .99) {
    t = 0.0f;
    this->last_path_t = 0.0f;
  }

  Vec2 path_current = this->path? this->path->sample_position(t) : Vec2 { 0,0 };
  Vec2 path_next    = this->path? this->path->sample_position(t+EPS) : Vec2 { 0,0 };
  Vec2 gradient = (path_next-path_current);

  Vec2 path_future = this->path? this->path->sample_position(t+this->lookahead*this->robot.get_velocity().length()) : Vec2 { 0,0 };
  
  Vec2 to_future = path_future - path_current;
  to_future *= 1.0 / to_future.length();

  Vec2 vel = robot.get_velocity();
  vel *= 1.0 / vel.length();

  float cos_theta = Vec2::dot(to_future, vel);

  if (cos_theta < 0.0001 || cos_theta != cos_theta) cos_theta = 1.0;

  spdlog::info("{}", powf(cos_theta, 16.0));

  gradient *= 8.0 * powf(cos_theta, 16.0) / gradient.length();

  float angle_setpoint = 0.0;

  this->target = path_current;

  this->time += dt;

  Vec2 pos = this->robot.get_frame_center();
  float angle = this->robot.get_rotation_radians();

  this->position_pid.kD = 10.0 * this->robot.get_velocity().length();

  Vec2 velocity_setpoint = this->position_pid.update(path_current, pos, dt) + gradient * this->feedforward * (t > .0001);
  float angular_velocity_setpoint = this->angle_pid.update(angle_setpoint, angle, dt);

  this->robot.set_velocity_setpoint(velocity_setpoint);
  this->robot.set_angular_velocity_setpoint(angular_velocity_setpoint);
}

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
}

}
