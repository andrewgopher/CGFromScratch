#include "geometry.h"

float Vector3f::dot(Vector3f v) {
    return x * v.x + y * v.y + z * v.z;
}

Vector3f Vector3f::cross(Vector3f v) {
    return Vector3f({y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x});
}

Vector3f operator-(Vector3f v1, Vector3f v2) {
    return Vector3f({v1.x - v2.x, v1.y - v2.y, v1.z - v2.z});
}

Vector3f operator*(Vector3f v, float s) {
    return Vector3f({v.x * s, v.y * s, v.z * s});
}

Vector3f operator*(float s, Vector3f v) {
    return Vector3f({v.x * s, v.y * s, v.z * s});
}

Vector3f operator+(Vector3f v1, Vector3f v2) {
    return Vector3f({v1.x + v2.x, v1.y + v2.y, v1.z + v2.z});
}

Triangle::Triangle(Vector3f arg_point1, Vector3f arg_point2, Vector3f arg_point3) {
    point1 = arg_point1;
    point2 = arg_point2;
    point3 = arg_point3;
    edge12 = point2 - point1;
    edge23 = point3 - point2;
    edge31 = point1 - point3;
    normal = (point3 - point1).cross(point2 - point1);
}

bool Triangle::contains_point(Vector3f point) {
    float lr1 = edge12.cross(point - point1).dot(normal);
    float lr2 = edge23.cross(point - point2).dot(normal);
    float lr3 = edge31.cross(point - point3).dot(normal);
    return lr1 < 0 && lr2 < 0 && lr3 < 0;
}

Plane::Plane(Vector3f arg_point, Vector3f arg_normal) {
    point = arg_point;
    normal = arg_normal;
    point_normal_dot = point.dot(normal);
}

Plane::Plane(Triangle triangle) {
    point = triangle.point1;
    Vector3f segment1 = triangle.point3 - triangle.point1;
    Vector3f segment2 = triangle.point2 - triangle.point1;
    normal = segment1.cross(segment2);
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
    return (line.point.dot(plane.normal) + plane.point_normal_dot) / plane_normal_line_slope_dot;
}

float intersects(Line line, Triangle triangle) {
    float possible_intersection = intersects(line, Plane(triangle));
    if (possible_intersection == std::numeric_limits<float>::max()) {
        return std::numeric_limits<float>::max();
    }
    if (triangle.contains_point(line.get_point(possible_intersection))) {
        return possible_intersection;
    } else {
        return std::numeric_limits<float>::max();
    }
}
