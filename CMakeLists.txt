cmake_minimum_required(VERSION 3.12)

project(frc-pathgen
  VERSION 0.1
  LANGUAGES C CXX
)

find_package(PkgConfig)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(SDL2 REQUIRED sdl2)
  include_directories(${SDL2_INCLUDE_DIRS})
  link_directories(${SDL2_LIBRARY_DIRS})
  add_definitions(${SDL2_CFLAGS_OTHER})
else()
  message(FATAL_ERROR "pkg-config not found")
endif()

add_subdirectory(${CMAKE_SOURCE_DIR}/impl)

add_executable(frc-pathgen ${FRC_PATHGEN_SOURCES})
target_include_directories(frc-pathgen PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(frc-pathgen PRIVATE ${SDL2_LIBRARIES})
