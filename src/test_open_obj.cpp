#include "open_obj.h"
#include <iostream>

int main() {
    Model3D test_model("assets/cube.obj");
    std::cout << "vertices: \n";
    for (auto vertex : test_model.vertices) {
        std::cout << vertex.x << " " << vertex.y << " " << vertex.z << "\n";
    }
    std::cout << "faces: \n";
    for (auto face : test_model.faces) {
        std::cout << face.point1.x << " " << face.point1.y << " " << face.point1.z << " | "
            << face.point2.x << " " << face.point2.y << " " << face.point2.z << " | "
            << face.point3.x << " " << face.point3.y << " " << face.point3.z << "\n";
    }
    return 0;
}