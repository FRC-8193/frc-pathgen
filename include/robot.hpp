/*
* frc-pathgen/include/robot.hpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#pragma once

#include <math.h>
#include <SDL2/SDL.h>
#include "vec2.hpp"
#include "viewport.hpp"
#include "pid.hpp"

namespace frc_pathgen {

class Robot {
public:
  Robot() {}

  inline Vec2 forward() const {
    return Vec2 { cosf(this->rotation_radians), sinf(this->rotation_radians) };
  }
  inline Vec2 right() const {
    return Vec2 { sinf(this->rotation_radians), -cosf(this->rotation_radians) };
  }

    // TODO: some better drawing utility stuff (wrap SDL_Renderer)
  void draw(SDL_Renderer *renderer, const Viewport &viewport);

  void set_velocity_setpoint(Vec2 velocity); // in m/s
  void set_angular_velocity_setpoint(float angular_velocity); // in rad/s
  
  void tick(float dt);
private:
  Vec2 frame_center = { 0,0 };
  Vec2 velocity = { 0,0 };
  float rotation_radians = 0.0;
  float angular_velocity = 0.0;

  Vec2 velocity_setpoint = { 0,0 };
  PIDController<Vec2, float> velocity_pid { 1.0f, 0.0f, 0.0f };

  float angular_velocity_setpoint = 0.0;
  PIDController<float> angular_velocity_pid { 5.0f, 0.1f, 0.0f };

  // applies the given *wheel torques* (PID outputs)
  void apply_voltages(Vec2 xy_voltage, float angular_voltage, float dt);

  static constexpr float mass = 50.0; // kg
  static constexpr float wheelbase = 60.0; // cm (side length)
  static constexpr float wheel_dist = 42.0; // cm
  static constexpr float moi = 3.0; // kg-m^2

  static constexpr float wheel_torque = 35.0; // Ncm
  static constexpr float wheel_radius = 4.375; // cm
 
  static constexpr float wheel_torque_m = wheel_torque / 100.0f; // Nm
  static constexpr float wheel_radius_m = wheel_radius / 100.0f; // m
  static constexpr float wheel_dist_m   = wheel_dist / 100.0f;   // m

  static constexpr float wheel_ground_force = wheel_torque_m / wheel_radius_m; // N
  static constexpr float wheel_ground_torque = wheel_ground_force * wheel_dist_m; // Nm

  // not a perfect model (should really be doing kinematics)
  static constexpr float bot_acceleration = 4.0 * wheel_ground_force / mass; // m/s^2
  static constexpr float bot_angular_acceleration = 4.0 * wheel_ground_torque / moi; // rad/s^2
};
}
