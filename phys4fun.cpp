// Dear ImGui: standalone example application for SDL2 + DirectX 11
// (SDL is a cross-platform general purpose library for handling windows, inputs, OpenGL/Vulkan/Metal graphics context creation, etc.)
// If you are new to Dear ImGui, read documentation from the docs/ folder + read the top of imgui.cpp.
// Read online: https://github.com/ocornut/imgui/tree/master/docs

#include <stdio.h> // printf
#include <string>
#include <vector>

#include <SDL2/SDL.h>

#include "ImSDL2.hpp"
#include "fun_physics.hpp"

// Main code
int main(int, char **)
{
    // Setup SDL
    // (Some versions of SDL before <2.0.10 appears to have performance/stalling issues on a minority of Windows systems,
    // depending on whether SDL_INIT_GAMECONTROLLER is enabled or disabled.. updating to latest version of SDL is recommended!)
    if (ImSDL2::InitSDL(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0)
    {
        printf("Error: %s\n", SDL_GetError());
        return -1;
    }

    // Setup window
    SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    SDL_Window *window = SDL_CreateWindow("phys4fun demo", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 800, 600, window_flags);

    int w, h;
    SDL_GetWindowSize(window, &w, &h);

    ImSDL2::CreateContext(window);

    ImGuiIO &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;     // Enable Docking

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    ImGuiStyle &style = ImGui::GetStyle();

    ImGuiWindowFlags win_flags = ImGuiWindowFlags_NoCollapse;
    win_flags |= ImGuiWindowFlags_NoTitleBar;
    win_flags |= ImGuiWindowFlags_NoResize;
    win_flags |= ImGuiWindowFlags_NoMove;
    win_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus;
    win_flags |= ImGuiWindowFlags_NoNavFocus;

    fun::phys::world world{10.f};
    std::vector<ImVec2> pos_logs;

    // vars related to the object creator
    float mass = 1.f;
    ImVec2 size{64.f, 64.f};
    ImVec4 color{1.f, 0.f, 0.f, 1.f};

    // vars related to the application's state
    bool done = false;
    bool vsync = true;
    bool filled = false;
    float fps = 60.f, rfps = 60.f;
    int running = 0;
    ImVec4 clear_color{0.45f, 0.55f, 0.60f, 1.00f};
    SDL_Event event;
    const char *run_btn[]{"Run", "Pause"};

    // world.add_object({
    //     .aabb = {
    //         .min = {0.f, 500.f},
    //         .max = {900.f, 600.f},
    //     },
    //     .inv_mass = 0.f,
    //     .color = {1.f, 1.f, 1.f, 1.f},
    //     .state = fun::rigid_body::fixed,
    // });

    while (!done)
    {
        Uint64 last = SDL_GetPerformanceCounter();

        while (SDL_PollEvent(&event))
        {
            done = ImSDL2::ProcessEvent(window, &event);

            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_RESIZED && event.window.windowID == SDL_GetWindowID(window))
                SDL_GetWindowSize(window, &w, &h);
        }

        // Start the Dear ImGui frame
        ImSDL2::NewFrame(window);

        ImGui::SetNextWindowSize(ImVec2{w * 1.f, h * 1.f});
        ImGui::SetNextWindowPos(ImVec2{});

        {
            // the "window" holding the docked windows
            ImGui::Begin("Scene", nullptr, win_flags);

            // move extra 20 pixels if running, so that the button
            ImGui::SetCursorPosX(w * .5f - 30.f - running * 10.f);

            if (ImGui::Button(run_btn[running]))
                running = 1 - running;

            // Create a dockspace
            ImGui::DockSpace(ImGui::GetID("MainDockspace"));

            ImGui::End();
        }

        // logging console
        {
            ImGui::Begin("Console");

            for (auto const &[x, y] : pos_logs)
                ImGui::Text("[INFO] Created object at (%f, %f).", x, y);

            ImGui::End();
        }

        {
            // the control panel
            ImGui::Begin("Control Panel");

            if (ImGui::CollapsingHeader("Config",
                                        ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::Checkbox("##hasfixedfps", &vsync);
                ImGui::SameLine();
                vsync |= ImGui::SliderFloat("Framerate", &fps, 10.f, 240.f);
                ImGui::Checkbox("Filled shapes", &filled);

                ImGui::SliderFloat("Gravity", &world.gravity, 1.f, 500.f);

                if (ImGui::Button("Reset"))
                {
                    world.clear();
                    pos_logs.clear();
                }
            }

            ImGui::NewLine();

            if (ImGui::CollapsingHeader("Create Object",
                                        ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::SliderFloat("Mass", &mass, .1f, 100.f);
                ImGui::SliderFloat2("Size", &size.x, 0.f, 200.f);
                ImGui::ColorEdit3("Color", &color.x);
            }

            ImGui::End();
        }

        {
            // the window where the simulation will be visualized
            ImGui::Begin("Viewport");

            ImVec2 pos = ImGui::GetCursorScreenPos();

            ImVec2 wMin = ImGui::GetWindowContentRegionMin();
            ImVec2 wMax = ImGui::GetWindowContentRegionMax();

            wMin.x += ImGui::GetWindowPos().x;
            wMin.y += ImGui::GetWindowPos().y;
            wMax.x += ImGui::GetWindowPos().x;
            wMax.y += ImGui::GetWindowPos().y;

            if (ImVec2 mouse = io.MousePos;
                io.MouseClicked[0] && (wMin.x < mouse.x) && (mouse.x < wMax.x))
                if ((wMin.y < mouse.y) && (mouse.y < wMax.y))
                {
                     world.add_object({
                        .aabb = {
                            .min = {mouse.x - size.x * .5f,
                                    mouse.y - size.y * .5f},
                            .max = {mouse.x + size.x * .5f,
                                    mouse.y + size.y * .5f},
                        },
                        .inv_mass = 1.f / mass,
                        .color = color,
                    });

                    pos_logs.emplace_back(mouse.x, mouse.y);
                }

            for (auto const &id : world.id)
            {
                auto const &obj = world.object[id];
                ImVec2 const start{
                    obj.aabb.min.x + pos.x,
                    obj.aabb.min.y + pos.y};

                ImVec2 const finish{
                    obj.aabb.max.x + pos.x,
                    obj.aabb.max.y + pos.y};

                if (filled)
                    ImGui::GetWindowDrawList()->AddRectFilled(
                        start, finish, ImColor(obj.color));
                else
                    ImGui::GetWindowDrawList()->AddRect(
                        start, finish, ImColor(obj.color));

                if (running)
                {
                    world.integrate(id, 1.f / rfps);

                    for (auto const &id2 : world.id)
                        world.colliding(id, id2);

                    while (!world.collisions.empty())
                    {
                        auto const &coll = world.collisions.top();
                        world.resolve(coll);
                        world.collisions.pop();
                    }
                }

                if (obj.aabb.min.x > wMax.x || obj.aabb.min.y > wMax.y)
                    if (obj.aabb.max.x < wMin.x || obj.aabb.max.y < wMin.y)
                    {
                        world.remove_object(id);
                        pos_logs.erase(pos_logs.cbegin() + id);
                    }
            }

            ImGui::End();
        }

        ImSDL2::Render(window, clear_color, false);

        float delta = float((SDL_GetPerformanceCounter() - last) / (double)SDL_GetPerformanceFrequency());

        if (vsync)
        {
            rfps = fps;
            if (float const dfps{1000.f / fps}; delta > dfps)
                SDL_Delay(Uint32(delta - dfps));
        }
        else
            rfps = 1.f / delta;
    }

    // Cleanup
    ImSDL2::DestroyContext();
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}