cmake_minimum_required(VERSION 3.0.0)

project(phys4fun
    VERSION 0.1.0
    LANGUAGES CXX)

find_package(imgui CONFIG REQUIRED)
find_package(SDL2 CONFIG REQUIRED)

add_executable(phys4fun
    phys4fun.cpp
    ImSDL2.hpp ImSDL2.cpp
    fun_physics.hpp fun_physics.cpp)

target_link_libraries(phys4fun
    PRIVATE
    imgui::imgui
    SDL2::SDL2 SDL2::SDL2main)
target_compile_features(phys4fun
    PRIVATE cxx_std_20)