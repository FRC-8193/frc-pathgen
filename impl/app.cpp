/*
* frc-pathgen/impl/app.cpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#include <spdlog/spdlog.h>
#include "app.hpp"
#include "world.hpp"

namespace frc_pathgen {

static const unsigned int WIDTH  = 800;
static const unsigned int HEIGHT = 600;

static int inst_count = 0;

App::App() : camera_controller(this->viewport) {
  this->window = nullptr;

  if (inst_count == 0) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
      spdlog::error("SDL could not initialize! SDL_Error: {}", SDL_GetError());
      return;
    }
    if (TTF_Init() == -1) {
      spdlog::warn("SDL_TTF could not initialize! SDL_Error: {}", TTF_GetError());
    }
    inst_count++;
  }

  this->window = SDL_CreateWindow("frc-pathgen",
    SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
    WIDTH, HEIGHT, SDL_WINDOW_SHOWN);

  if (!this->window) {
    spdlog::error("Window could not be created! SDL_Error: {}", SDL_GetError());
    SDL_Quit();
    return;
  }

  this->renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
  this->viewport.width = WIDTH;
  this->viewport.height = HEIGHT;

    // TODO: make this relative to exe dir
  this->grid_font = TTF_OpenFont("assets/JetBrainsMono-Regular.ttf", 14);
}

void App::run() {
  if (!this->is_ok()) {
    spdlog::error("Attepmted to run an App in an invalid state!");
    return;
  }

  bool running = true;
  SDL_Event e;

  this->viewport.units_per_vw = 250;

  while (running) {
    while (SDL_PollEvent(&e)) {
      if (this->camera_controller.consume_event(e)) continue;
      if (e.type == SDL_QUIT) running = false;
    }

    SDL_SetRenderDrawColor(this->renderer, 16, 16, 16, 255);
    SDL_RenderClear(this->renderer);

    draw_world_gridlines(this->renderer, this->grid_font, this->viewport);

    this->robot.draw(this->renderer, this->viewport); // TODO: tick

    SDL_RenderPresent(this->renderer);
  }

  this->teardown();
}

void App::teardown() {
  if (!this->is_ok()) return;
  SDL_DestroyRenderer(this->renderer);
  SDL_DestroyWindow(this->window);
  if ((--inst_count) == 0) {
    SDL_Quit();
  }
}
}
