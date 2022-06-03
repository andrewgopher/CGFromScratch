#include "geometry.h"
#include "float_limits_def.h"
#include <cmath>
#include <iterator>

Vector3f::Vector3f(float arg_x, float arg_y, float arg_z) {
    x = arg_x;
    y = arg_y;
    z = arg_z;
}


float Vector3f::dot(Vector3f v) {
    return x * v.x + y * v.y + z * v.z;
}

Vector3f Vector3f::cross(Vector3f v) {
    return Vector3f(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
}

std::string Vector3f::to_string() {
    return std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z);
}

Vector3f operator-(Vector3f v1, Vector3f v2) {
    return Vector3f(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}

Vector3f operator*(Vector3f v, float s) {
    return Vector3f(v.x * s, v.y * s, v.z * s);
}

Vector3f operator*(float s, Vector3f v) {
    return Vector3f(v.x * s, v.y * s, v.z * s);
}

Vector3f operator+(Vector3f v1, Vector3f v2) {
    return Vector3f(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}

Vector3f operator*(Vector3f v, Matrix3f m) {
    return Vector3f(v.x * m[0][0] + v.y * m[0][1] + v.z * m[0][2], v.x * m[1][0] + v.y * m[1][1] + v.z * m[1][2], v.x * m[2][0] + v.y * m[2][1] + v.z * m[2][2]);
}

Vector3f operator/(Vector3f v, float f) {
    return Vector3f(v.x / f, v.y / f, v.z / f);
}

Vector3f operator/(Vector3f v, int i) {
    return Vector3f(v.x / i, v.y / i, v.z / i);
}

float Vector3f::get_length() {
    return sqrt(x * x + y * y + z * z);
}

bool Vector3f::all_less_than(Vector3f v) {
    return x < v.x && y < v.y && z < v.z;
}

bool operator<(Vector3f v1, Vector3f v2) {
    return v1.x < v2.x && v1.y < v2.y && v1.z < v2.z;
}

Triangle::Triangle(Vector3f arg_point1, Vector3f arg_point2, Vector3f arg_point3) {
    point1 = arg_point1;
    point2 = arg_point2;
    point3 = arg_point3;

    recompute();
}

bool Triangle::contains_point(Vector3f& point) {
    Vector3f v2 = point - point1;

    float dot02 = edge13.dot(v2);
    float dot12 = edge12.dot(v2);

    float inv_denom = 1.0f / (edge13_edge13_dot * edge12_edge12_dot - edge13_edge12_dot * edge13_edge12_dot);

    float area1 = (edge12_edge12_dot * dot02 - edge13_edge12_dot * dot12) * inv_denom;
    float area2 = (edge13_edge13_dot * dot12 - edge13_edge12_dot * dot02) * inv_denom;

    return area1 >= 0 && area2 >= 0 && area1 <= 1 && area2 <= 1 && area1 + area2 <= 1;
}

std::string Triangle::to_string() {
    return point1.to_string() + " | " + point2.to_string() + " | " + point3.to_string();
}

Triangle Triangle::transform(Matrix3f transformation) {
    return Triangle(point1 * transformation, point2 * transformation, point3 * transformation);
}

void Triangle::transform_in_place(Matrix3f transformation) {
    point1 = point1 * transformation;
    point2 = point2 * transformation;
    point3 = point3 * transformation;
    recompute();
}

void Triangle::translate(Vector3f& translation) {
    point1 = point1 + translation;
    point2 = point2 + translation;
    point3 = point3 + translation;
    recompute();
}

void Triangle::recompute() {
    edge12 = point2 - point1;
    edge23 = point3 - point2;
    edge31 = point1 - point3;

    edge21 = point1 - point2;
    edge32 = point2 - point3;
    edge13 = point3 - point1;

    normal = (edge12).cross(edge13);
    edge13_edge13_dot = edge13.dot(edge13);
    edge13_edge12_dot = edge13.dot(edge12);
    edge12_edge12_dot = edge12.dot(edge12);

    vertices[0] = point1;
    vertices[1] = point2;
    vertices[2] = point3;

    line12.point = point1;
    line23.point = point2;
    line31.point = point3;

    line12.slope = point2 - point1;
    line23.slope = point3 - point2;
    line31.slope = point1 - point3;
}

TriangleFace::TriangleFace(Vector3f arg_point1, Vector3f arg_point2, Vector3f arg_point3, Vector3f normal1,
                           Vector3f normal2, Vector3f normal3) {
    triangle = Triangle(arg_point1, arg_point2, arg_point3);
    normals[0] = normal1;
    normals[1] = normal2;
    normals[2] = normal3;
}

TriangleFace::TriangleFace(Vector3f arg_point1, Vector3f arg_point2, Vector3f arg_point3) {
    triangle = Triangle(arg_point1, arg_point2, arg_point3);
}

std::string  TriangleFace::to_string() {
    return triangle.to_string() + " | " + normals[0].to_string() + " | " + normals[1].to_string() + " | " + normals[2].to_string();
}

Plane::Plane(Vector3f arg_point, Vector3f arg_normal) {
    point = arg_point;
    normal = arg_normal;
    point_normal_dot = point.dot(normal);
}

Plane::Plane(Triangle triangle) {
    point = triangle.point1;
    normal = (triangle.point3 - triangle.point1).cross(triangle.point2 - triangle.point1);
    point_normal_dot = point.dot(normal);
}

Line::Line(Vector3f point1, Vector3f point2) {
    point = point1;
    slope = point2 - point1;
}

Vector3f Line::get_point(float param) {
    return point + param * slope;
}

float intersects(Line& line, Plane& plane) {
    float plane_normal_line_slope_dot = plane.normal.dot(line.slope);
    if (plane_normal_line_slope_dot == 0) {
        return FLOAT_MAX;
    } else {
        return (-line.point.dot(plane.normal) + plane.point_normal_dot) / plane_normal_line_slope_dot;
    }
}


float intersects(Line& line, Triangle& triangle) { //credit: https://stackoverflow.com/a/42752998/12620352 second part
    float det = -line.slope.dot(triangle.normal);
    float inv_det = 1.0f / det;
    Vector3f AO = line.point - triangle.point1;
    Vector3f DAO = AO.cross(line.slope);
    float u = triangle.edge13.dot(DAO) * inv_det;
    float v = -triangle.edge12.dot(DAO) * inv_det;
    float t = AO.dot(triangle.normal) * inv_det;
    return (det >= 1e-6 && t >= 0 && u >= 0 && v >= 0 && (u + v) <= 1) ? t : FLOAT_MAX;
}

Matrix3f::Matrix3f(std::array<std::array<float, 3>, 3> arg_arr) {
    arr = arg_arr;
}

std::array<float, 3> Matrix3f::operator[](int index) {
    return arr[index];
}

void Matrix3f::set_x_rotation(float rotation) {
    arr[0][0] = cos(rotation);
    arr[0][1] = -sin(rotation);
    arr[0][2] = 0;
    arr[1][0] = sin(rotation);
    arr[1][1] = cos(rotation);
    arr[1][2] = 0;
    arr[2][0] = 0;
    arr[2][1] = 0;
    arr[2][2] = 1;
}


void Matrix3f::set_y_rotation(float rotation) {
    arr[0][0] = cos(rotation);
    arr[0][1] = 0;
    arr[0][2] = sin(rotation);
    arr[1][0] = 0;
    arr[1][1] = 1;
    arr[1][2] = 0;
    arr[2][0] = -sin(rotation);
    arr[2][1] = 0;
    arr[2][2] = cos(rotation);
}


void Matrix3f::set_z_rotation(float rotation) {
    arr[0][0] = 1;
    arr[0][1] = 0;
    arr[0][2] = 0;
    arr[1][0] = 0;
    arr[1][1] = cos(rotation);
    arr[1][2] = -sin(rotation);
    arr[2][0] = 0;
    arr[2][1] = sin(rotation);
    arr[2][2] = cos(rotation);
}

Matrix3f operator*(Matrix3f m1, Matrix3f m2) {
    std::array<std::array<float, 3>, 3> new_arr = {{
                                                    {{m1[0][0] * m2[0][0] + m1[0][1] * m2[1][0] + m1[0][2] * m2[2][0], m1[0][0] * m2[0][1] + m1[0][1] * m2[1][1] + m1[0][2] * m2[2][1], m1[0][0] * m2[0][2] + m1[0][1] * m2[1][2] + m1[0][2] * m2[2][2]}},
                                                    {{m1[1][0] * m2[0][0] + m1[1][1] * m2[1][0] + m1[1][2] * m2[2][0], m1[1][0] * m2[0][1] + m1[1][1] * m2[1][1] + m1[1][2] * m2[2][1], m1[1][0] * m2[0][2] + m1[1][1] * m2[1][2] + m1[1][2] * m2[2][2]}},
                                                    {{m1[2][0] * m2[0][0] + m1[2][1] * m2[1][0] + m1[2][2] * m2[2][0], m1[2][0] * m2[0][1] + m1[2][1] * m2[1][1] + m1[2][2] * m2[2][1], m1[2][0] * m2[0][2] + m1[2][1] * m2[1][2] + m1[2][2] * m2[2][2]}}
                                                    }};
    return Matrix3f(new_arr);
}

AABB::AABB(Vector3f arg_min, Vector3f arg_max) {
    min = arg_min;
    max = arg_max;
    recompute();
}

void AABB::recompute() {
    min_x_plane = Plane(min, Vector3f(max.x - min.x, 0, 0));
    min_y_plane = Plane(min, Vector3f(0, max.y - min.y, 0));
    min_z_plane = Plane(min, Vector3f(0, 0, max.z - min.z));
    max_x_plane = Plane(max, Vector3f(min.x - max.x, 0, 0));
    max_y_plane = Plane(max, Vector3f(0, min.y - max.y, 0));
    max_z_plane = Plane(max, Vector3f(0, 0, min.z - max.z));


    planes[0] = min_x_plane;
    planes[1] = min_y_plane;
    planes[2] = min_z_plane;
    planes[3] = max_x_plane;
    planes[4] = max_y_plane;
    planes[5] = max_z_plane;
    vertices[0] = min;
    vertices[1] = Vector3f(min.x, min.y, max.z);
    vertices[2] = Vector3f(min.x, max.y, min.z);
    vertices[3] = Vector3f(min.x, max.y, max.z);
    vertices[4] = Vector3f(max.x, min.y, min.z);
    vertices[5] = Vector3f(max.x, min.y, max.z);
    vertices[6] = Vector3f(max.x, max.y, min.z);
    vertices[7] = max;
}

bool AABB::contains(Vector3f point) {
    return (min - Vector3f(1e-5, 1e-5, 1e-5)) < point && point < (max + Vector3f(1e-5, 1e-5, 1e-5));
}

bool in_between(float a, float b, float c) {
    return (a >= b && a <= c) || (a <= b && a >= c);
}

bool is_intersects(Line& line, AABB& aabb) {
    for (auto& plane : aabb.planes) {
        float current_intersection = intersects(line, plane);
        if (current_intersection != FLOAT_MAX && aabb.contains(line.get_point(current_intersection))) {
            return true;
        }
    }
    return false;
}

std::string AABB::to_string() {
    return min.to_string() + " | " + max.to_string();
}

std::array<float, 2> intersects(Line& line, AABB& aabb) {
    std::array<float, 2> result = {FLOAT_MAX, FLOAT_MAX};
    int i = 0;
    for (auto& plane : aabb.planes) {
        float current_intersection = intersects(line, plane);
        if (current_intersection != FLOAT_MAX && aabb.contains(line.get_point(current_intersection))) {
            result[i] = current_intersection;
            i++;
            i%=2;
        }
    }
    return result;
}

float project(Vector3f& point, Line& line) {
    return (line.point - point).dot(line.slope) / line.slope.dot(line.slope);
}

bool separates(Triangle& triangle, AABB& aabb, Vector3f axis) {
    float min_aabb_project = FLOAT_MAX;
    float max_aabb_project = FLOAT_MIN;
    float min_triangle_project = FLOAT_MAX;
    float max_triangle_project = FLOAT_MIN;

    for (int i = 0; i < 8; i++) {
        float current_projection = aabb.vertices[i].dot(axis);
        if (current_projection < min_aabb_project) {
            min_aabb_project = current_projection;
        }
        if (current_projection > max_aabb_project) {
            max_aabb_project = current_projection;
        }
    }

    for (int i = 0; i < 3; i++) {
        float current_projection = triangle.vertices[i].dot(axis);
        if (current_projection < min_triangle_project) {
            min_triangle_project = current_projection;
        }
        if (current_projection > max_triangle_project) {
            max_triangle_project = current_projection;
        }
    }
    return (max_aabb_project < min_triangle_project || max_triangle_project < min_aabb_project);
}

bool is_intersects(Triangle& triangle, AABB& aabb) {
#define separate(axis) separates(triangle, aabb, axis)
    return !separate(unit_x_vector) && !separate(unit_y_vector) && !separate(unit_z_vector) && !separate(triangle.normal)
        && !separate(unit_x_vector.cross(triangle.edge12)) && !separate(unit_x_vector.cross(triangle.edge23))
        && !separate(unit_x_vector.cross(triangle.edge31)) && !separate(unit_y_vector.cross(triangle.edge12))
        && !separate(unit_y_vector.cross(triangle.edge23)) && !separate(unit_y_vector.cross(triangle.edge31))
        && !separate(unit_z_vector.cross(triangle.edge12)) && !separate(unit_z_vector.cross(triangle.edge23))
        && !separate(unit_z_vector.cross(triangle.edge31));
#undef separate
}