#include <iostream>
#include "geometry.h"

int main() {
    Line line({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
    Triangle triangle({2.0, -1.0, -1.0}, {2.0, -1.0, 1.0}, {2.0, 1.0, 0});
    std::cout << intersects(line, triangle) << std::endl;
    return 0;
}
