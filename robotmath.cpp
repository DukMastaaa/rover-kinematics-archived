#include "robotmath.h"

#include <math.h>
#include <iostream>


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

Matrix3f vecToSkewSym(const Vector3f& vector) {
    return (
        Matrix3f() <<
            0, - vector[2], vector[1],
            vector[2], 0, - vector[0],
            - vector[1], vector[0], 0
    ).finished(); 
}


Vector3f skewSymToVec(const Matrix3f& matrix) {
    return (
        Vector3f() << matrix(2, 1), matrix(0, 2), matrix(1, 0)
    ).finished();
}


Matrix4f twistToMatrixRepr(const Twist& twist) {
    // [V] = [ [w], v; 0, 0 ] given twist V = [w; v].
    Matrix4f matrix = Matrix4f::Zero();
    matrix.topLeftCorner(3, 3) = vecToSkewSym(twist.angularPart());
    matrix.topRightCorner(3, 1) = twist.linearPart();
    return matrix;
}


Twist matrixReprToTwist(const Matrix4f& matrix) {
    // see formula in `twistToMatrixRepr`
    Twist twist;
    twist.angularPart() = skewSymToVec(matrix.topLeftCorner(3, 3));
    twist.linearPart() = matrix.topRightCorner(3, 1);
    return twist;
}


Matrix6f adjointRepr(const Affine3D& transformation) {
    // [Ad_T] =
    // [
    //      R,    the 3x3 zero matrix;
    //      [p]R, R
    // ]
    // given that T = [ R, p; 0, 1 ] as usual
    Matrix6f adjoint = Matrix6f::Zero();
    adjoint.topLeftCorner(3, 3) = transformation.rotation();
    adjoint.bottomRightCorner(3, 3) = transformation.rotation();
    adjoint.bottomLeftCorner(3, 3) = 
        vecToSkewSym(transformation.translation()) * transformation.rotation();
    return adjoint;
}


Matrix3f matrixExpRotation(float angle, const Vector3f& axis) {
    // Rodruiges' Rotation Formula
    // e^{ [w]theta } = I + [w]sin(theta) + [w]^2 (1 - cos(theta))
    Matrix3f skewRepr = vecToSkewSym(axis);
    Matrix3f rotation = Matrix3f::Identity()
        + std::sin(angle) * skewRepr 
        + (1 - std::cos(angle)) * (skewRepr * skewRepr);

    return rotation;
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
        Matrix3f skewAngular = vecToSkewSym(screw.angularPart());

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
        const Vector6f& screw = screwAxes.col(i);
        output = output * matrixExpScrew(vec_theta[i], screw);
    }
    return output;
}


Jacobian spaceJacobian(const JointAngleVector& vec_theta, const ScrewArray& screwAxes) {
    // J_s = [ J_s1 ... J_sn ] where n is number of joints and
    // J_s1 = S1,
    // J_si = [Ad_M] S_i for i = 2, ..., n; and
    // M = e^{ [S1]theta1 } ... e^{ [S_{i-1}]theta_{i-1} }
    Jacobian jacobian;
    jacobian.col(1) = screwAxes.col(1);
    
    Affine3D M = Affine3D::Identity();

    for (int i = 1; i < jointCount; i++) {
        const Vector6f& screw = screwAxes.col(i-1);
        M = M * matrixExpScrew(vec_theta[i-1], screw);
        jacobian.col(i) = adjointRepr(M) * screwAxes.col(i);
    }
    return jacobian;
}


Jacobian bodyJacobian(const JointAngleVector& vec_theta, const ScrewArray& screwAxes) {
    // J_b = [ J_b1 ... J_bn ] where n is number of joints and
    // J_bn = Bn,
    // J_bi = [Ad_M] B_i for i = 1, ..., n-1; and
    // M = e^{ - [Bn]theta_n } ... e^{ - [B_{i+1}]theta_{i+1} }.
    // Negative exponent indicates matrix inverse.
    Jacobian jacobian;
    jacobian.col(jointCount - 1) = screwAxes.col(jointCount - 1);
    // for (int i = 2; i < jointCount; i++) {
    //     Affine3D M = Affine3D::Identity();
    //     for (int j = 0; j < i; j++) {
    //         const Vector6f& screw = screwAxes.col(j);
    //         M = M * matrixExpScrew(vec_theta[j], screw);
    //     }
    //     jacobian.col(i) = adjointRepr(M) * screwAxes.col(i);
    // }
    return jacobian;
}
