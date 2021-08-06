#include <iostream>

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "robotmath.h"

using Eigen::Vector3f;
using Eigen::Matrix3f;


int main() {
    // std::cout << "Make a new transformation matrix that is just the identity:\n";
    // Affine3D myTransformation = Affine3D::Identity();
    // std::cout << myTransformation.matrix() << '\n';

    // std::cout << "Now we translate the matrix:\n";
    // myTransformation.translate(Vector3f(1, 2, 3));
    // std::cout << myTransformation.matrix() << '\n';

    // std::cout << "Rotate by some axis:\n";
    // float angle = 0.5 * EIGEN_PI;
    // Vector3f axis(1, 1, 1);
    // axis.normalize();

    Twist screw;
    screw << 0, 0, 1, 0, -2, 0;
    Affine3D trans = matrixExpTwist(0.5 * EIGEN_PI, screw);
    std::cout << trans.matrix() << '\n';
}