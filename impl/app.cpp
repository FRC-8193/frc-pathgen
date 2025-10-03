/*
* frc-pathgen/include/app.cpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#include "app.hpp"
#include <spdlog/spdlog.h>

namespace frc_pathgen {

static const unsigned int WIDTH  = 800;
static const unsigned int HEIGHT = 600;

static int inst_count = 0;

App::App() {
  this->window = nullptr;

  if (inst_count == 0) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
      spdlog::error("SDL could not initialize! SDL_Error: {}", SDL_GetError());
      return;
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
}

void App::run() {
  if (!this->is_ok()) {
    spdlog::error("Attepmted to run an App in an invalid state!");
    return;
  }

  bool running = true;
  SDL_Event e;

  Vec2 objects[] = {
      {-1, -1}, {1, -1}, {-1, 1}, {1,1}
  };

  this->viewport.units_per_vw = 5.5;

  while (running) {
    while (SDL_PollEvent(&e)) {
      if (this->process_event(e)) continue;
      if (e.type == SDL_QUIT) running = false;
    }

    SDL_SetRenderDrawColor(this->renderer, 100, 149, 237, 255);
    SDL_RenderClear(renderer);


    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    for (auto& obj : objects) {
        Vec2 px = this->viewport.world_to_px(obj);
        SDL_Rect r{int(px.x - 5), int(px.y - 5), 10, 10};
        SDL_RenderFillRect(renderer, &r);
    }

    SDL_RenderPresent(renderer);
  }

  this->teardown();
}

bool App::process_event(const SDL_Event &e) {
  switch (e.type) {
    default:
      return false;
  }
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
