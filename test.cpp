#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>

int main() {
    Eigen::Vector2f v(1, 0);
    std::cout << v * 2;

    return 0;
}