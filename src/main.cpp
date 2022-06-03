#include <SDL.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <array>
#include "geometry.h"
#include "open_obj.h"
#include "float_limits_def.h"

float fast_minf(float a, float b) {
    return a < b ? a : b;
}

float fast_maxf(float a, float b) {
    return a > b ? a : b;
}

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

    void translate(Vector3f& translation) {
        triangle.translate(translation);
    }
};

struct Octree {
    Octree* children[8] = {nullptr};
    std::vector<int> triangle_indices;
    AABB container;
    int depth = 1;
    const static int max_depth = 6;


    Octree(AABB arg_container, int arg_depth) {
        container = arg_container;
        depth = arg_depth;
    };

    void divide() {
#define cmin container.min
#define cmax container.max
        children[0] = new Octree(AABB(cmin, cmax / 2), depth + 1);
        children[1] = new Octree(AABB({cmin.x, cmin.y, cmax.z / 2}, {cmax.x / 2, cmax.y / 2, cmax.z}), depth + 1);
        children[2] = new Octree(AABB({cmin.x, cmax.y / 2, cmin.z}, {cmax.x / 2, cmax.y, cmax.z / 2}), depth + 1);
        children[3] = new Octree(AABB({cmin.x, cmax.y / 2, cmax.z / 2}, {cmax.x / 2, cmax.y, cmax.z}), depth + 1);
        children[4] = new Octree(AABB({cmax.x / 2, cmin.y, cmin.z}, {cmax.x, cmax.y / 2, cmax.z / 2}), depth + 1);
        children[5] = new Octree(AABB({cmax.x / 2, cmin.y, cmax.z / 2}, {cmax.x, cmax.y / 2, cmax.z}), depth + 1);
        children[6] = new Octree(AABB({cmax.x / 2, cmax.y / 2, cmin.z}, {cmax.x, cmax.y , cmax.z / 2}), depth + 1);
        children[7] = new Octree(AABB(cmax / 2, cmax), depth + 1);
#undef cmin
#undef cmax
    };

    void add_triangles(std::vector<TriangleObject>& triangles, std::vector<int>& indices) {

        for (auto& index : indices) {
            if (is_intersects(triangles[index].triangle, container)) {
                triangle_indices.push_back(index);
            }
        }


        if (depth < triangle_indices.size() && depth < max_depth) {
            if (children[0] == nullptr) divide();
            for (auto& child : children) {
                child->add_triangles(triangles, triangle_indices);
            }
        }
    }

    void add_triangles(std::vector<TriangleObject>& triangles) {
        int i = 0;
        for (auto& triangle : triangles) {
            if (is_intersects(triangle.triangle, container)) {
                triangle_indices.push_back(i);
            }
            i++;
        }

        if (depth < triangle_indices.size() && depth < max_depth) {
            if (children[0] == nullptr) divide();
            for (auto& child : children) {
                child->add_triangles(triangles, triangle_indices);
            }
        }
    }


    std::pair<float, int> intersect(std::vector<TriangleObject>& triangles, Line& line) { //run reset_visited() on first call (non-recursive call)
        if (intersects(line, container)[0] != FLOAT_MAX) {
            if (children[0] == nullptr) {
                float closest_intersection = FLOAT_MAX;
                int closest_intersection_triangle_index = -1;
                for (auto& triangle_index : triangle_indices) {
                    float current_intersection = intersects(line, triangles[triangle_index].triangle);
                    if (current_intersection < closest_intersection) {
                        closest_intersection = current_intersection;
                        closest_intersection_triangle_index = triangle_index;
                    }
                }
                return {closest_intersection, closest_intersection_triangle_index};
            } else {
                float closest_intersection = FLOAT_MAX;
                int closest_intersection_triangle_index = -1;
                for (auto& child : children) {
                    if (is_intersects(line, child->container) && child->triangle_indices.size() > 0) {
                        std::pair<float, int> current_intersection = child->intersect(triangles, line);
                        if (current_intersection.first < closest_intersection) {
                            closest_intersection = current_intersection.first;
                            closest_intersection_triangle_index = current_intersection.second;
                        }
                    }
                }
                return {closest_intersection, closest_intersection_triangle_index};
            }
        } else {
            return {FLOAT_MAX, -1};
        }
    }
};

struct Mesh {
    std::vector<TriangleObject> triangles;
    AABB bounding_box;
    Octree* octree = nullptr;
    Mesh(std::vector<TriangleFace>& faces) {
        for (auto& face : faces) {
            triangles.push_back({face.triangle, face.normals, Color({255, 255, 255, 255})});
        }
        compute_bounding_box();
        compute_octree();
    }
    void compute_bounding_box() {
        float min_x = FLOAT_MAX;
        float min_y = FLOAT_MAX;
        float min_z = FLOAT_MAX;
        float max_x = FLOAT_MIN;
        float max_y = FLOAT_MIN;
        float max_z = FLOAT_MIN;

        int i = 0;
        for (auto& triangle : triangles) {
            for (auto point : triangle.triangle.vertices) {
                min_x = fast_minf(min_x, point.x);
                min_y = fast_minf(min_y, point.y);
                min_z = fast_minf(min_z, point.z);
                max_x = fast_maxf(max_x, point.x);
                max_y = fast_maxf(max_y, point.y);
                max_z = fast_maxf(max_z, point.z);
            }
            i++;
        }

        bounding_box = AABB(Vector3f(min_x, min_y, min_z), Vector3f(max_x, max_y, max_z));
    }
    void transform_in_place(Matrix3f& transformation) {
        for (auto& triangle : triangles) {
            triangle.transform_in_place(transformation);
        }
        compute_bounding_box();
        compute_octree();
    }

    void translate(Vector3f translation) { //TODO: homogenous coordinates to just use transform_in_place instead
        for (auto& triangle :triangles) {
            triangle.translate(translation);
        }
        //TODO: no need to completely recompute; just translate
        compute_bounding_box();
        compute_octree();
    }

    void compute_octree() {
        std::cout << "computing octree...\n";
        delete octree;
        octree = new Octree(bounding_box, 1);
        octree->add_triangles(triangles);
        std::cout << "finished computing octree\n";
    }

    std::pair<float, int> intersect(Line line) {
        return octree->intersect(triangles, line);
    }
};

struct PointLight {
    Vector3f position;
    float brightness;
};

struct AmbientLight {
    float brightness;
};

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

float camera_width = 8;
float camera_height = 4.5;
Vector3f camera_position = {0, 2, -10};
float viewport_distance = 5;
//TODO: camera pointing direction


std::vector<Mesh> meshes;
std::vector<PointLight> point_lights;
std::vector<AmbientLight> ambient_lights;
Matrix3f teapot_y_rotation;
Matrix3f teapot_x_rotation;
Matrix3f teapot_z_rotation;

Matrix3f teapot_transformation;

Matrix3f icosahedron_transformation;

unsigned long long sum_render_time = 0;
int num_frames = 0;

Uint8 random_0_255() {
    return rand() % 256;
}

Color random_color() {
    return Color({random_0_255(), random_0_255(), random_0_255(), 255});
}

void define_scene() {
    std::cout << "defining scene/loading objects...\n";
    srand(time(NULL));
    Model3D teapot("assets/teapot.obj");
    meshes.emplace_back(Mesh(teapot.faces));

    teapot_y_rotation.set_y_rotation(0.05f);
    teapot_x_rotation.set_x_rotation(0.05f);
    teapot_z_rotation.set_z_rotation(0.05f);

    teapot_transformation = teapot_x_rotation * teapot_y_rotation * teapot_z_rotation;

    point_lights.push_back({Vector3f(5, 5, 0), 1.5});
    ambient_lights.push_back({0.3});

    Model3D icosahedron("assets/icosahedron.obj");
    meshes.emplace_back(Mesh(icosahedron.faces));
    meshes[1].translate(Vector3f(-5, 0, 0));


    std::cout << "finished defining scene\n";
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
    for (auto& light : point_lights) {
        Vector3f point_to_light = light.position - point;
        float point_to_light_normal_dot = point_to_light.dot(normal);
        if (point_to_light_normal_dot > 0) {
            brightness += light.brightness * (point_to_light_normal_dot / (normal.get_length() * point_to_light.get_length()));
        }
    }
    for (auto& light : ambient_lights) {
        brightness += light.brightness;
    }
    return brightness;
}

Color trace_ray(Line line) {
    float closest_intersection = FLOAT_MAX;
    int closest_intersection_mesh_index = -1;
    int closest_intersection_triangle_index = -1;
    int i = 0;
    for (auto& mesh : meshes) {
        std::pair<float, int> current_intersection = mesh.intersect(line);
        if (current_intersection.first < closest_intersection && current_intersection.first > 1) {
            closest_intersection = current_intersection.first;
            closest_intersection_mesh_index = i;
            closest_intersection_triangle_index = current_intersection.second;
        }
        i ++;
    }
    if (closest_intersection_mesh_index == -1) {
        return Color({0, 0, 0, 0});
    } else {
        return meshes[closest_intersection_mesh_index].triangles[closest_intersection_triangle_index].color * compute_lighting(line.get_point(closest_intersection), meshes[closest_intersection_mesh_index].triangles[closest_intersection_triangle_index].triangle.normal);
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
    float frame_rate = 60.0f;
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
        ticks_count = SDL_GetTicks();
        
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

        meshes[0].transform_in_place(teapot_transformation); //TODO:
        meshes[1].translate(Vector3f((num_frames - 1) % 41 < 20 ? 0.05f : -0.05f, 0, 0));
    }
    SDL_Quit();
    return 0;
}
