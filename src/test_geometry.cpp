#include <iostream>
#include "geometry.h"

int main() {

    Matrix3f m1({{{{1, 2, 3}}, {{4, 5, 6}}, {{7, 8, 9}}}});

    Matrix3f m2({{{{1, 2, 1}}, {{2, 4, 6}}, {{7, 2, 5}}}});

    Matrix3f m3;
    m3 = m1 * m2;
    std::cout << (m1 * m2)[0][0] << std::endl;

    std::cout << m3[0][1] << std::endl;
    return 0;
}
