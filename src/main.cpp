#include <SDL.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <ctime>
#include <cstdlib>
#include <array>
#include "geometry.h"
#include "open_obj.h"

struct Color {
    Uint8 r;
    Uint8 g;
    Uint8 b;
    Uint8 a;
};

struct TriangleObject {
    Triangle triangle;
    std::array<Vector3f, 3> normals;
    Color color;
    void transform_in_place(Matrix3f transformation) {
        triangle.transform_in_place(transformation);
        for (int i = 0; i < 3; i ++) {
            normals[i] = normals[i] * transformation;
        }
    }
};

struct PointLight {
    Vector3f position;
    float brightness;
};

struct AmbientLight {
    float brightness;
};

float fast_minf(float a, float b) {
    return a < b ? a : b;
}

Color operator*(Color c, float brightness) {
    Color out;
    out.r = fast_minf(c.r * brightness, 255);
    out.g = fast_minf(c.g * brightness, 255);
    out.b = fast_minf(c.b * brightness, 255);
    out.a = fast_minf(c.a * brightness, 255);
    return out;
}


SDL_Renderer* renderer;
const int window_width = 800;
const int window_height = 450;

Color screen[window_width][window_height];

int camera_width = 16;
int camera_height = 9;
Vector3f camera_position = {0, 0, -5};
float viewport_distance = 4.8;
//TODO: camera pointing direction


std::vector<TriangleObject> triangles;
std::vector<PointLight> point_lights;
std::vector<AmbientLight> ambient_lights;
Matrix3f y_rotation;
Matrix3f x_rotation;
Matrix3f z_rotation;

Matrix3f transformation;

unsigned long long sum_render_time = 0;
float num_frames = 0;

Uint8 random_0_255() {
    return rand() % 256;
}

Color random_color() {
    return Color({random_0_255(), random_0_255(), random_0_255(), 255});
}

void define_scene() {
    srand(time(NULL));
    Model3D cube("assets/cube.obj");
    for (auto face : cube.faces) {
        triangles.push_back({face.triangle, face.normals, random_color()});
    }
    y_rotation.set_y_rotation(0.05f);
    x_rotation.set_x_rotation(0.05f);
    z_rotation.set_z_rotation(0.05f);

    transformation = x_rotation * y_rotation * z_rotation;

    point_lights.push_back({Vector3f(5, 5, 0), 1.5});
    ambient_lights.push_back({0.3});
}


void draw_point(int x, int y, Color color) {
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    SDL_RenderDrawPoint(renderer, x, y);
}

Vector3f window_to_viewport(float x, float y) {
    return Vector3f((x - window_width / 2) * camera_width / window_width, (window_height / 2 - y) * camera_height / window_height, viewport_distance) + camera_position;
}

std::pair<int, int> pixel_id(int id) {
    return {id % window_width, id / window_width};
}

float compute_lighting(Vector3f point, Vector3f normal) { //diffuse
    float brightness = 0;
    for (auto light : point_lights) {
        Vector3f point_to_light = light.position - point;
        float point_to_light_normal_dot = point_to_light.dot(normal);
        if (point_to_light_normal_dot > 0) {
            brightness += light.brightness * (point_to_light_normal_dot / (normal.get_length() * point_to_light.get_length()));
        }
    }
    for (auto light : ambient_lights) {
        brightness += light.brightness;
    }
    return brightness;
}

Color trace_ray(Line line) {
    float closest_intersection = std::numeric_limits<float>::max();
    int closest_intersection_index = -1;
    Vector3f closest_intersection_normal;
    int i = 0;
    for (auto triangle : triangles) {
        float current_intersection = intersects(line, triangle.triangle);
        if (current_intersection < closest_intersection && current_intersection > 1) {
            closest_intersection = current_intersection;
            closest_intersection_index = i;
            closest_intersection_normal = triangle.triangle.normal;
        }
        i += 1;
    }
    if (closest_intersection_index == -1) {
        return Color({0, 0, 0, 0});
    } else {
        return triangles[closest_intersection_index].color * compute_lighting(line.get_point(closest_intersection), closest_intersection_normal);
    }
}

void render(int pixel_start, int pixel_end) { //[pixel_start, pixel_end)
    for (int i = pixel_start; i < pixel_end; i += 1) {
        std::pair<int, int> pixel = pixel_id(i);
        Color color_at_point = trace_ray(Line(camera_position, window_to_viewport(pixel.first, pixel.second)));
        screen[pixel.first][pixel.second] = color_at_point;
    }
}

void blit() {
    for (int i = 0; i < window_width; i += 1) {
        for (int j = 0; j < window_height; j += 1) {
            draw_point(i, j, screen[i][j]);
        }
    }
    draw_point(window_width / 2, window_height / 2, {0, 255, 0, 255});
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
    unsigned int cpu_count = std::thread::hardware_concurrency();
    while (running) {
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
        SDL_RenderClear(renderer);
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
                break;
            }
        }
        while (!SDL_TICKS_PASSED(SDL_GetTicks(), ticks_count + 1000.0f / frame_rate));
        
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        if (cpu_count == 0) {
            render(0, window_width * window_height);
        } else {
            int curr_pixel_id = 0;
            std::vector<std::thread> threads;
            for (int i = 0; i < cpu_count; i += 1) {
                std::thread curr_thread = std::thread(render, curr_pixel_id, curr_pixel_id + window_width * window_height / cpu_count);
                threads.push_back(std::move(curr_thread));
                curr_pixel_id += window_width * window_height / cpu_count;
            }
            for (auto& thread : threads) {
                if (thread.joinable()) {
                    thread.join();
                }
            }
            if (window_height * window_width != curr_pixel_id) {
                render(curr_pixel_id, window_width * window_height);
            }
        }
        blit();
        num_frames ++;
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Time to render = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]";
        sum_render_time += std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

        std::cout << " avg = " << (double) sum_render_time / num_frames << "[ms]" << std::endl;
        
        SDL_RenderPresent(renderer);

        for (auto& triangle : triangles) {
            triangle.transform_in_place(transformation);
        }
    }
    SDL_Quit();
    return 0;
}
