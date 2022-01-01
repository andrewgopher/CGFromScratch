#include <SDL.h>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Core>
#include <chrono>
#include "geometry.h"

struct Color {
    Uint8 r;
    Uint8 g;
    Uint8 b;
    Uint8 a;
};

struct TriangleObject {
    Triangle triangle;
    Color color;
};


SDL_Renderer* renderer;
int window_width = 800;
int window_height = 450;

int camera_width = 16;
int camera_height = 9;
Vector3f camera_position = {10, 0, 0};
float viewport_distance = 1;
//TODO: camera pointing direction


std::vector<TriangleObject> triangles;
//TODO: triangle object struct/class

void define_scene() {
    triangles.push_back({Triangle({-3, -3, 5}, {3, -3, 5}, {-3, 3, 5}), Color({255, 0, 0, 255})});
}


void draw_point(int x, int y, Color color) {
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    SDL_RenderDrawPoint(renderer, x, y);
}

Vector3f window_to_viewport(float x, float y) {
    return Vector3f({(x - window_width / 2) * camera_width / window_width, (window_height / 2 - y) * camera_height / window_height, viewport_distance}) + camera_position;
}

Color trace_ray(Line line) {
    float closest_intersection = std::numeric_limits<float>::max();
    int closest_intersection_index = -1;
    int i = 0;
    for (auto triangle : triangles) {
        float current_intersection = intersects(line, triangle.triangle);
        if (current_intersection < closest_intersection && closest_intersection > 1) {
            closest_intersection = current_intersection;
            closest_intersection_index = i;
        }
        i += 1;
    }
    if (closest_intersection_index == -1) {
        return Color({0, 0, 0, 0});
    } else {
        return triangles[closest_intersection_index].color;
    }
    
}

void render() {
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    for (int i = 0; i < window_width; i += 1) {
        for (int j = 0; j < window_height; j += 1) {
            Color color_at_point = trace_ray(Line(camera_position, window_to_viewport(i, j)));
            draw_point(i, j, color_at_point);
        }
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time to render = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
}

int main() {
    define_scene();
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "2");
    int sdl_result = SDL_Init(SDL_INIT_VIDEO);
    if (sdl_result != 0) {
        SDL_Log("SDL initialization error: %s", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    SDL_Window* window = SDL_CreateWindow(
        "CG From Scratch",
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, window_width, window_height, SDL_WINDOW_SHOWN
    );
    if (!window) {
        SDL_Log("Window creation error: %s", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    renderer = SDL_CreateRenderer(
        window,
        -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
    );
    bool running = true;
    Uint32 ticks_count = 0;
    float frame_rate = 60.0;
    while (running) {
        SDL_SetRenderDrawColor(
            renderer,
            0,
            0,
            0,
            0
        );
        SDL_RenderClear(renderer);
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
                break;
            }
        }
        while (!SDL_TICKS_PASSED(SDL_GetTicks(), ticks_count + 1000.0f / frame_rate));
        render();
        SDL_RenderPresent(renderer);
    }
    SDL_Quit();
    return 0;
}