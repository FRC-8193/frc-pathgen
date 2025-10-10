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

namespace frc_pathgen {

PathFollower::PathFollower(Robot &robot) : robot(robot), 
  position_pid(20.0, 0.0, 2.0), angle_pid(7.5, 0.0, 1.5) {
}

void PathFollower::set_path(Path &path) {
  this->duration = sqrtf(path.max_acceleration() / Robot::bot_acceleration) * 1.25; // Keep the robot running as hard as possible

  this->path = &path;

  spdlog::info("Calculated duration {} {}", this->duration, path.max_acceleration());
}

void PathFollower::draw(SDL_Renderer *renderer, const Viewport &viewport) {
  Vec2 tp = viewport.world_to_px(this->target);

  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  draw_filled_circle(renderer, tp.x, tp.y, 10);

  ImGui::Begin("Path Follower Controls");
  ImGui::SliderFloat("Velocity feedforward", &this->feedforward, 0.0f, 1.0f);
  ImGui::End();
}

void PathFollower::tick(float dt) {
  bool start = fmodf(this->time / this->duration, 2.0) < 1.0f;

  static const float EPS = .001;

  Vec2 path_current = this->path? this->path->sample_position(start*fmodf(this->time/this->duration, 1.0f)) : Vec2 { 0,0 };
  Vec2 path_next    = this->path? this->path->sample_position(start*fmodf(this->time/this->duration, 1.0f)+EPS) : Vec2 { 0,0 };
  Vec2 gradient = (path_next-path_current) / EPS;

  float angle_setpoint = 0.0;

  this->target = path_current;

  this->time += dt;

  Vec2 pos = this->robot.get_frame_center();
  float angle = this->robot.get_rotation_radians();

  this->position_pid.kD = 15.0 * this->robot.get_velocity().length();

  Vec2 velocity_setpoint = this->position_pid.update(path_current, pos, dt) + gradient * this->feedforward;
  float angular_velocity_setpoint = this->angle_pid.update(angle_setpoint, angle, dt);

  this->robot.set_velocity_setpoint(velocity_setpoint);
  this->robot.set_angular_velocity_setpoint(angular_velocity_setpoint);
}
}
