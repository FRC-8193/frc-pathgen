/*
* frc-pathgen/impl/app.cpp
* Copyright (c) 2025 Frederick Ziola et al. (New Lothrop Robotics)
* Licensed under MIT. see LICENSE file in the repository root.
*   Use, copy, modify, and distribute as needed, simply credit the original author.
*   Because we are programmers, not lawyers!
*/

#include <spdlog/spdlog.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>
#include "app.hpp"
#include "SDL_timer.h"
#include "world.hpp"

namespace frc_pathgen {

static const unsigned int WIDTH  = 1920;
static const unsigned int HEIGHT = 1080;

App::App() : robot(), camera_controller(this->viewport, &this->robot) {
  this->window = nullptr;

  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    spdlog::error("SDL could not initialize! SDL_Error: {}", SDL_GetError());
    return;
  }
  

  if (TTF_Init() == -1) {
    spdlog::warn("SDL_TTF could not initialize! SDL_Error: {}", TTF_GetError());
  }

  this->window = SDL_CreateWindow("frc-pathgen",
    SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
    WIDTH, HEIGHT, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);

  if (!this->window) {
    spdlog::error("Window could not be created! SDL_Error: {}", SDL_GetError());
    SDL_Quit();
    return;
  }

  this->renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
  this->viewport.width = WIDTH;
  this->viewport.height = HEIGHT;

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();

  // SDL window + SDL_Renderer
  ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
  ImGui_ImplSDLRenderer2_Init(renderer);

    // TODO: make this relative to exe dir
  this->grid_font = TTF_OpenFont("assets/JetBrainsMono-Regular.ttf", 14);
}

void App::run() {
  this->robot.set_angular_velocity_setpoint(1);
  this->robot.set_velocity_setpoint({0, 1});
  if (!this->is_ok()) {
    spdlog::error("Attepmted to run an App in an invalid state!");
    return;
  }

  bool running = true;
  SDL_Event e;

  this->viewport.units_per_vw = 5;

  Uint64 perf_freq = SDL_GetPerformanceFrequency();
  Uint64 last_time = SDL_GetPerformanceCounter();
  
  Uint64 print_time = last_time;

  while (running) {
    Uint64 time = SDL_GetPerformanceCounter();

    float dt = (float)(time - last_time) / perf_freq; // seconds
    last_time = time;
   
    if ((time - print_time) / perf_freq > 1.0) {
      print_time = time;
      spdlog::info("{}fps", (int)(1.0 / dt));
    }

    ImGuiIO &io = ImGui::GetIO();

    while (SDL_PollEvent(&e)) {
      ImGui_ImplSDL2_ProcessEvent(&e);
      if (io.WantCaptureKeyboard || io.WantCaptureMouse) continue;
      if (this->camera_controller.consume_event(e)) continue;
      if (e.type == SDL_WINDOWEVENT &&
        e.window.event == SDL_WINDOWEVENT_RESIZED) {
        int w = e.window.data1;
        int h = e.window.data2;

        SDL_RenderSetLogicalSize(renderer, w, h);
        SDL_RenderSetViewport(renderer, NULL);

        this->viewport.width = w;
        this->viewport.height = h;
      }
      if (e.type == SDL_QUIT) running = false;
    }

    // Physics stuff
    this->robot.tick(dt);
    this->camera_controller.tick(dt);

    // Rendering

    SDL_SetRenderDrawColor(this->renderer, 16, 16, 16, 255);
    SDL_RenderClear(this->renderer);

    draw_world_gridlines(this->renderer, this->grid_font, this->viewport);

    
    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    this->robot.draw(this->renderer, this->viewport);
    this->camera_controller.draw(this->renderer, this->viewport);

    ImGui::Render();
    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), this->renderer);

    SDL_RenderPresent(this->renderer);
  }

  this->teardown();
}

void App::teardown() {
  if (!this->is_ok()) return;
  SDL_DestroyRenderer(this->renderer);
  SDL_DestroyWindow(this->window);
  SDL_Quit();
}
}
