#ifndef CGFROMSCRATCH_GEOMETRY_H
#define CGFROMSCRATCH_GEOMETRY_H
#include <limits>
#include <iostream>
#include <array>
#include "float_limits_def.h"

//TODO: Vector4f for homogeneous coordinates

struct Vector3f {
    Vector3f() {};
    Vector3f(float arg_x, float arg_y, float arg_z);
    float x;
    float y;
    float z;
    float dot(Vector3f v);
    float get_length();
    Vector3f cross(Vector3f v);
    std::string to_string();
};

bool operator<(Vector3f v1, Vector3f v2);

static Vector3f zero_vector = Vector3f(0, 0, 0);
static Vector3f unit_x_vector = Vector3f(1, 0, 0);
static Vector3f unit_y_vector = Vector3f(0,1,0);
static Vector3f unit_z_vector = Vector3f(0,0,1);
static Vector3f min_vector = Vector3f(FLOAT_MIN, FLOAT_MIN, FLOAT_MIN);

static Vector3f max_vector = Vector3f(FLOAT_MAX, FLOAT_MAX, FLOAT_MAX);

struct Matrix3f {
    Matrix3f() {};
    Matrix3f(std::array<std::array<float, 3>, 3> arg_arr);
    void set_z_rotation(float rotation);
    void set_y_rotation(float rotation);
    void set_x_rotation(float rotation);
    std::array<float, 3> operator[](int index);
    std::array<std::array<float, 3>, 3> arr = {};
};

Vector3f operator-(Vector3f v1, Vector3f v2);
Vector3f operator*(Vector3f v, float s);
Vector3f operator*(float s, Vector3f v);
Vector3f operator+(Vector3f v1, Vector3f v2);
Vector3f operator/(Vector3f v, float f);
Vector3f operator/(Vector3f v, int i);

Vector3f operator*(Vector3f v, Matrix3f m);
Matrix3f operator*(Matrix3f m1, Matrix3f m2);



struct Line {
    Line() {};
    Line(Vector3f point1, Vector3f point2);
    Vector3f get_point(float param);
    Vector3f point;
    Vector3f slope;
};

struct Triangle {
    Triangle() {};
    Triangle(Vector3f arg_point1, Vector3f arg_point2, Vector3f arg_point3);
    bool contains_point(Vector3f& point);
    std::string to_string();
    Triangle transform(Matrix3f transformation);
    void transform_in_place(Matrix3f transformation);
    void translate(Vector3f& translation);
    void recompute();
    Vector3f point1;
    Vector3f point2;
    Vector3f point3;
    Vector3f edge12;
    Vector3f edge23;
    Vector3f edge31;
    Vector3f edge21;
    Vector3f edge32;
    Vector3f edge13;
    Vector3f normal;

    float edge13_edge13_dot;
    float edge13_edge12_dot;
    float edge12_edge12_dot;

    Line line12;
    Line line23;
    Line line31;

    std::array<Vector3f, 3> vertices;
};

struct TriangleFace {
    TriangleFace(Vector3f arg_point1, Vector3f arg_point2, Vector3f arg_point3, Vector3f normal1, Vector3f normal2,
                 Vector3f normal3);
    TriangleFace(Vector3f arg_point1, Vector3f arg_point2, Vector3f arg_point3);
    std::string to_string();
    Triangle triangle;
    std::array<Vector3f, 3> normals;
};

struct Plane {
    Plane() {};
    Plane(Vector3f arg_point, Vector3f arg_normal);
    Plane(Triangle triangle);
    Vector3f point;
    Vector3f normal;
    float point_normal_dot;
};

struct AABB {
    AABB() {};
    AABB(Vector3f arg_min, Vector3f arg_max);
    void recompute();
    bool contains(Vector3f point);
    std::string to_string();
    Vector3f min = {0, 0, 0};
    Vector3f max = {0, 0, 0};
    Plane min_x_plane;
    Plane min_y_plane;
    Plane min_z_plane;
    Plane max_x_plane;
    Plane max_y_plane;
    Plane max_z_plane;

    std::array<Plane, 6> planes;
    std::array<Vector3f, 8> vertices;
};

float project(Vector3f& point, Line& line);

bool in_between(float a, float b, float c);

bool separates(Triangle& triangle, AABB& aabb, Vector3f axis);
float intersects(Line& line, Triangle& triangle);
float intersects(Line& line, Plane& plane);
bool is_intersects(Line& line, AABB& aabb);
std::array<float, 2> intersects(Line& line, AABB& aabb);
bool is_intersects(Triangle& triangle, AABB& aabb);

#endif
