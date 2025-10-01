/*
* frc-pathgen/impl/main.cpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#include <SDL2/SDL.h>
#include <iostream>
#include <spdlog/spdlog.h>

int main(int argc, char **argv) {
  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    spdlog::error("SDL could not initialize! SDL_Error: {}", SDL_GetError());
    return 1;
  }

  SDL_Window* window = SDL_CreateWindow("SDL2 Hello World",
    SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
    640, 480, SDL_WINDOW_SHOWN);

  if (!window) {
    spdlog::error("Window could not be created! SDL_Error: ", SDL_GetError());
    SDL_Quit();
    return 1;
  }

  SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

  bool running = true;
  SDL_Event e;

  while (running) {
    while (SDL_PollEvent(&e)) {
        if (e.type == SDL_QUIT) running = false;
    }

    SDL_SetRenderDrawColor(renderer, 100, 149, 237, 255); // cornflower blue
    SDL_RenderClear(renderer);
    SDL_RenderPresent(renderer);
  }

  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
