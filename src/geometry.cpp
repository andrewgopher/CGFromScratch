#include "geometry.h"
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

float Vector3f::get_length() {
    return sqrt(x * x + y * y + z * z);
}

Triangle::Triangle(Vector3f arg_point1, Vector3f arg_point2, Vector3f arg_point3) {
    point1 = arg_point1;
    point2 = arg_point2;
    point3 = arg_point3;
    edge12 = point2 - point1;
    edge23 = point3 - point2;
    edge31 = point1 - point3;

    edge21 = point1 - point2;
    edge32 = point2 - point3;
    edge13 = point3 - point1;

    normal = (point3 - point1).cross(point2 - point1);
}

bool Triangle::contains_point(Vector3f point) {
    Vector3f v2 = point - point1;

    float dot00 = edge13.dot(edge13);
    float dot01 = edge13.dot(edge12);
    float dot02 = edge13.dot(v2);
    float dot11 = edge12.dot(edge12);
    float dot12 = edge12.dot(v2);

    float inv_denom = 1 / (dot00 * dot11 - dot01 * dot01);

    float area1 = (dot11 * dot02 - dot01 * dot12) * inv_denom;
    float area2 = (dot00 * dot12 - dot01 * dot02) * inv_denom;

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

void Triangle::recompute() {
    edge12 = point2 - point1;
    edge23 = point3 - point2;
    edge31 = point1 - point3;

    edge21 = point1 - point2;
    edge32 = point2 - point3;
    edge13 = point3 - point1;

    normal = (point3 - point1).cross(point2 - point1);
}

TriangleFace::TriangleFace(Vector3f arg_point1, Vector3f arg_point2, Vector3f arg_point3, Vector3f normal1,
                           Vector3f normal2, Vector3f normal3) {
    triangle = Triangle(arg_point1, arg_point2, arg_point3);
    normals[0] = normal1;
    normals[1] = normal2;
    normals[2] = normal3;
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

float intersects(Line line, Plane plane) {
    float plane_normal_line_slope_dot = plane.normal.dot(line.slope);
    if (plane_normal_line_slope_dot == 0) {
        return std::numeric_limits<float>::max();
    }
    return (-line.point.dot(plane.normal) + plane.point_normal_dot) / plane_normal_line_slope_dot;
}

float intersects(Line line, Triangle triangle) {
    float possible_intersection = intersects(line, Plane(triangle.point1, triangle.normal));
    if (possible_intersection == std::numeric_limits<float>::max()) {
        return std::numeric_limits<float>::max();
    }
    if (triangle.contains_point(line.get_point(possible_intersection))) {
        return possible_intersection;
    } else {
        return std::numeric_limits<float>::max();
    }
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