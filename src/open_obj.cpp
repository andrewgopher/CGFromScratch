#include "open_obj.h"

void Model3D::open(std::string file_name) { //TODO
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
            std::array<int, 3> point_indices;
            std::array<int, 3> normal_indices;
            std::array<int, 3> tex_indices;
            bool has_points = false;
            bool has_normals = false;
            bool has_texes = false;
            for (int i = 0; i < 3; i ++) {
                std::string curr_vertex, curr_index;
                fin >> curr_vertex;
                std::stringstream ss(curr_vertex);
                int j = 0;
                while (std::getline(ss, curr_index, '/')) {
                    if (curr_index.empty()) {
                        j ++;
                        continue;
                    }
                    if (j == 0) {
                        has_points = true;
                        point_indices[i] = std::stoi(curr_index);
                    } else if (j == 1) {
                        has_texes = true;
                        tex_indices[i] = std::stoi(curr_index);
                    } else if (j == 2) {
                        has_normals = true;
                        normal_indices[i] = std::stoi(curr_index);
                    }
                    j ++;
                }
            }
            if (has_points && has_normals) {
                faces.emplace_back(vertices[point_indices[0] - 1], vertices[point_indices[1] - 1], vertices[point_indices[2] - 1],
                               normals[normal_indices[0] - 1], normals[normal_indices[1] - 1], normals[normal_indices[2] - 1]);
            } else if (has_points) {
                faces.emplace_back(vertices[point_indices[0] - 1], vertices[point_indices[1] - 1], vertices[point_indices[2] - 1]);
            }
        } else if (curr_str == "vn") {
            Vector3f new_normal;
            fin >> new_normal.x >> new_normal.y >> new_normal.z;
            normals.push_back(new_normal);
        } else {
            fin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }
    std::cout << "opened " << file_name << "\n";
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
