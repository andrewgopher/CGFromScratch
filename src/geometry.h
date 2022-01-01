#ifndef CGFROMSCRATCH_GEOMETRY_H
#define CGFROMSCRATCH_GEOMETRY_H
#include <limits>
#include <iostream>

struct Vector3f {
    float x;
    float y;
    float z;
    float dot(Vector3f v);
    Vector3f cross(Vector3f v);
};

Vector3f operator-(Vector3f v1, Vector3f v2);
Vector3f operator*(Vector3f v, float s);
Vector3f operator*(float s, Vector3f v);
Vector3f operator+(Vector3f v1, Vector3f v2);

struct Triangle {
    Triangle() {};
    Triangle(Vector3f arg_point1, Vector3f arg_point2, Vector3f arg_point3);
    bool contains_point(Vector3f point);
    Vector3f point1;
    Vector3f point2;
    Vector3f point3;
    Vector3f edge12;
    Vector3f edge23;
    Vector3f edge31;
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