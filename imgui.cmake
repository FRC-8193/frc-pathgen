add_library(ImGui STATIC
  ${CMAKE_CURRENT_SOURCE_DIR}/extern/imgui/imgui.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/extern/imgui/imgui_draw.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/extern/imgui/imgui_tables.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/extern/imgui/imgui_widgets.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/extern/imgui/backends/imgui_impl_sdl2.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/extern/imgui/backends/imgui_impl_sdlrenderer2.cpp
)

target_include_directories(ImGui PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/extern/imgui/ ${CMAKE_CURRENT_SOURCE_DIR}/extern/imgui/backends)
target_link_libraries(ImGui PRIVATE ${SDL2_LIBRARIES})
