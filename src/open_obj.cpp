#include "open_obj.h"

void Model3D::open(std::string file_name) {
    vertices.clear();
    faces.clear();
    std::ifstream fin;
    fin.open(file_name);
    std::string curr_str;
    while (fin >> curr_str) {
        if (curr_str == "v") {
            Vector3f new_vertex;
            fin >> new_vertex.x >> new_vertex.y >> new_vertex.z;
            vertices.push_back(new_vertex);
        } else if (curr_str == "f") {
            Vector3f point1, point2, point3;
            int point1_index, point2_index, point3_index;
            fin >> point1_index;
            fin >> point2_index;
            fin >> point3_index;
            faces.push_back(Triangle(vertices[point1_index - 1], vertices[point2_index - 1], vertices[point3_index - 1]));
        } else {
            fin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }
    std::cout << to_string();
}

Model3D::Model3D(std::string file_name) {
    open(file_name);
}

std::string Model3D::to_string() {
    std::string result;
    result += std::to_string(faces.size());
    result += '\n';
    for (auto face : faces) {
        result += face.to_string();
        result += '\n';
    }
    
    return result;
}
