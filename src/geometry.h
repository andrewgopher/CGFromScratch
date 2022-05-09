#ifndef CGFROMSCRATCH_GEOMETRY_H
#define CGFROMSCRATCH_GEOMETRY_H
#include <limits>
#include <iostream>
#include <array>

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
    bool all_less_than(Vector3f v);
};

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

Vector3f operator*(Vector3f v, Matrix3f m);
Matrix3f operator*(Matrix3f m1, Matrix3f m2);

struct Triangle {
    Triangle() {};
    Triangle(Vector3f arg_point1, Vector3f arg_point2, Vector3f arg_point3);
    bool contains_point(Vector3f point);
    std::string to_string();
    Triangle transform(Matrix3f transformation);
    void transform_in_place(Matrix3f transformation);
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

    std::array<Vector3f, 3> points;
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

struct Line {
    Line() {};
    Line(Vector3f point1, Vector3f point2);
    Vector3f get_point(float param);
    Vector3f point;
    Vector3f slope;
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
};

bool in_between(float a, float b, float c);

float intersects(Line line, Triangle triangle);
float intersects(Line line, Plane plane);
bool is_intersects(Line line, AABB aabb);

#endif
