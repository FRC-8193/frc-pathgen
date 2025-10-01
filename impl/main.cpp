/*
* frc-pathgen/impl/main.cpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#include "app.hpp"

int main(int argc, char **argv) {
  frc_pathgen::App app;

  if (!app.is_ok()) return -1;

  app.run();

  return 0;
}
