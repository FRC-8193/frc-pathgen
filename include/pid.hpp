/*
* frc-pathgen/include/pid.hpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#pragma once

template<typename T, typename K=T>
struct PIDController {
  PIDController(K kP, K kI, K kD) : kP(kP), kI(kI), kD(kD) {}

  K kP, kI, kD;

  T update(const T &target, const T &current, float dt) {
    T error = target - current;
    this->accum_error += error * dt;
    T diff_error = (error - this->last_error) / dt;
    this->last_error = error;

    return this->kP * error + this->kI * this->accum_error + this->kD * diff_error;
  }

  void reset() {
    this->last_error = this->accum_error = {};
  }
private:
  T last_error = {}, accum_error = {};
};
