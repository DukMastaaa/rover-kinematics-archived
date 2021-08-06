#include "robotmath.h"

#include <math.h>

using Eigen::Matrix3f;
using Eigen::Vector3f;


Matrix3f skewSymRepr(const Vector3f& vector) {
    return (
        Matrix3f() <<
            0, - vector[2], vector[1],
            vector[2], 0, - vector[0],
            - vector[1], vector[0], 0
    ).finished(); 
}


Matrix3f matrixExpRotation(float angle, const Vector3f& axis) {
    Matrix3f skewRepr = skewSymRepr(axis);

    // Rodruiges' Rotation Formula
    // e^{ [w]theta } = I + [w]sin(theta) + [w]^2 (1 - cos(theta))
    Matrix3f rotation = Matrix3f::Identity()
        + std::sin(angle) * skewRepr 
        + (1 - std::cos(angle)) * (skewRepr * skewRepr);

    return rotation;
}