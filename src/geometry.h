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

struct Triangle {
    Triangle() {};
    Triangle(Vector3f arg_point1, Vector3f arg_point2, Vector3f arg_point3);
    bool contains_point(Vector3f point);
    std::string to_string();
    Triangle transform(Matrix3f transformation);
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

float intersects(Line line, Triangle triangle);
float intersects(Line line, Plane plane);

#endif
