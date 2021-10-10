#include<cmath>
#include<Eigen/Core>
#include<Eigen/Dense>
#include<iostream>

int main()
{
    Eigen::MatrixXf start(3, 1);
    start << 2.0f, 1.0f, 1.0f;
    Eigen::Matrix3f transform;
    transform << 0.707, -0.707, 1.0, 0.707, 0.707, 2.0, 0.0, 0.0, 1.0;
    Eigen::MatrixXf out(3, 1);
    out = transform * start;
    std::cout << out;
    return 0;
}