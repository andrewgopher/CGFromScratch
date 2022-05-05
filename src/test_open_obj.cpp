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
        std::cout << face.to_string();
    }
    return 0;
}