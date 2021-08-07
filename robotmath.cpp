#include "robotmath.h"

#include <math.h>
#include <iostream>

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


Twist::Twist() {
    Vector6f();
}


Twist::Twist(Vector6f& vector) {
    // TODO: very hacky solution, i'm not sure if there are any better ways...
    Vector6f::operator=(vector);
}

Twist::Twist(const Vector6f& vector) {
    // TODO: AEUGH
    Vector6f::operator=(vector);
}


Eigen::VectorBlock<Vector6f, 3> Twist::angularPart() {
    return head<3>();
}


const Eigen::VectorBlock<const Vector6f, 3> Twist::angularPart() const {
    return head<3>();
}


Eigen::VectorBlock<Vector6f, 3> Twist::linearPart() {
    return tail<3>();
}


const Eigen::VectorBlock<const Vector6f, 3> Twist::linearPart() const {
    return tail<3>();
}


Affine3D matrixExpScrew(float angle, const Twist& screw) {
    Affine3D transformation = Affine3D::Identity();

    if (screw.angularPart().isZero()) {
        // e^{ [s]theta } = 
        // [
        //      I_3, theta * Sv;
        //      0,   1
        // ]
        transformation.translate(angle * screw.linearPart());
    } else {
        // e^{ [s]theta } = 
        // [
        //      e^{ [Sw]theta }, M Sv;
        //      0,               1
        // ]
        // where M = theta*I + (1 - cos(theta))[Sw] + (theta - sin(theta))[Sw]^2
        Matrix3f skewAngular = skewSymRepr(screw.angularPart());

        Matrix3f M = angle * Matrix3f::Identity()
            + (1 - std::cos(angle)) * skewAngular
            + (angle - std::sin(angle)) * (skewAngular * skewAngular);
        
        transformation.translate(M * screw.linearPart());

        Vector3f normalisedAngularAxis = screw.angularPart().normalized();

        // USING FUNCTIONS PROVIDED IN EIGEN (probably the better option):
        transformation.rotate(Eigen::AngleAxisf(angle, normalisedAngularAxis));
        // USING FUNCTIONS DEFINED HERE (alternate option, commented out):
        // transformation.matrix().topLeftCorner(3, 3) = matrixExpRotation(angle, normalisedAngularAxis);
    }
    return transformation;
}


Affine3D forwardKinematics(const JointAngleVector& vec_theta, const ScrewArray& screwAxes) {
    Affine3D output = Affine3D::Identity();
    for (int i = 0; i < jointCount; i++) {
        // this line required as you can't go through two implicit conversions at once
        const Vector6f& screwAxis = screwAxes.col(i);
        // for some reason, you can't multiply a transformation matrix by another one with *=,
        // even both of them have been declared as affine transformations, not projective...
        // you have to do *= with a normal matrix. oh well.
        output *= matrixExpScrew(vec_theta[i], screwAxis).matrix();
    }
    return output;
}
