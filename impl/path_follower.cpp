/*
* frc-pathgen/impl/path_follower.cpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#include "path_follower.hpp"

namespace frc_pathgen {

PathFollower::PathFollower(Robot &robot) : robot(robot), 
  position_pid(20.0, 0.0, 2.0), angle_pid(7.5, 0.0, 1.5) {
}

void PathFollower::draw(SDL_Renderer *renderer, const Viewport &viewport) {

}

void PathFollower::tick(float dt) {
  Vec2 pos_setpoint = { 0,0 };
  float angle_setpoint = 0.0;

  Vec2 pos = this->robot.get_frame_center();
  float angle = this->robot.get_rotation_radians();

  this->position_pid.kD = 10.0 * this->robot.get_velocity().length();

  Vec2 velocity_setpoint = this->position_pid.update(pos_setpoint, pos, dt);
  float angular_velocity_setpoint = this->angle_pid.update(angle_setpoint, angle, dt);

  this->robot.set_velocity_setpoint(velocity_setpoint);
  this->robot.set_angular_velocity_setpoint(angular_velocity_setpoint);
}
}
