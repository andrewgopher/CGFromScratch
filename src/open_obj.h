#ifndef CGFROMSCRATCH_OPEN_OBJ_H
#define CGFROMSCRATCH_OPEN_OBJ_H
#include "geometry.h"
#include <vector>


class Model3D {
public:
    Model3D(std::string file_name);
    void open(std::string file_name);
    std::string to_string();
    std::vector<Vector3f> vertices;
    std::vector<Vector3f> normals;
    std::vector<TriangleFace> faces;
};

#endif
