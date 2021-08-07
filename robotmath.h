#pragma once

#include "Eigen/Dense"
#include "Eigen/Geometry"

constexpr const int jointCount = 2;  // TODO: is there a better way to do this compared to defining it within the header file?

using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::Vector3f;

typedef Eigen::Matrix<float, 6, 1> Vector6f;  // technically not a Vector6f but close enough
typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef Eigen::Transform<float, 3, Eigen::TransformTraits::Affine> Affine3D;

typedef Eigen::Matrix<float, jointCount, 1> JointAngleVector;

// This is intended to package screw axes into a single matrix for easy storage.
typedef Eigen::Matrix<float, 6, jointCount> ScrewArray;
typedef Eigen::Matrix<float, jointCount, 6> ScrewArrayTranspose;

typedef Eigen::Matrix<float, 6, jointCount> Jacobian;


// Represents twists V = [w; v] and screws S = [Sw; Sv] in R6.
class Twist : public Vector6f {
    public:
        Twist();

        // We need to be able to automatically convert between Twist and Vector6f.
        // Twist to Vector6f is automatic (derived to base conversion).
        // Vector6f to Twist is provided here.
        Twist(Vector6f& vector);
        Twist(const Vector6f& vector);

        // first three elements
        Eigen::VectorBlock<Vector6f, 3> angularPart();
        const Eigen::VectorBlock<const Vector6f, 3> angularPart() const;

        // last three elements
        Eigen::VectorBlock<Vector6f, 3> linearPart();
        const Eigen::VectorBlock<const Vector6f, 3> linearPart() const;
};


// Returns the skew-symmetric representation [v] of a vector v in R3.
// This is defined such that for another vector w, [v]w = v x w
// where x denotes the cross product.
Matrix3f vecToSkewSym(const Vector3f& vector);


// Returns the vector v corresponding to the provided skew-symmetric representation [v].
Vector3f skewSymToVec(const Matrix3f& matrix);


// Returns the matrix representation [V] of a twist V expressed in either
// the body or space frame.
Matrix4f twistToMatrixRepr(const Twist& twist);


// Returns the twist V given its matrix representation [V].
Twist matrixReprToTwist(const Matrix4f& matrix);


// Returns the adjoint representation of a transformation matrix.
Matrix6f adjointRepr(const Affine3D& transformation);


// Calculates the rotation matrix (through matrix exponential) obtained
// by rotating about `axis` by `angle`. Warning: `axis` must be normalised!
// 
// This can also be done with Eigen using `Eigen::AngleAxisf`.
Matrix3f matrixExpRotation(float angle, const Vector3f& axis);


// Returns the transformation matrix (through matrix exponential) obtained
// by following `screw` by `angle`.
Affine3D matrixExpScrew(float angle, const Twist& screw);


// Returns the product of exponentials e^{ [S1]theta1 } e^{ [S2]theta2 } ...
// which can readily be multiplied by some initial configuration matrix M.
// Note that if screws are expressed in the body frame, post-multiply M 
// by the output. If screws are expressed in the space frame, pre-multiply
// M by the output.
Affine3D forwardKinematics(const JointAngleVector& vec_theta, const ScrewArray& screwAxes);


// Calculates the Jacobian matrix expressed in the space frame at the given joint angles.
Jacobian spaceJacobian(const JointAngleVector& vec_theta, const ScrewArray& screwAxes);


// Calculates the Jacobian matrix expressed in the body frame at the given joint angles.
Jacobian bodyJacobian(const JointAngleVector& vec_theta, const ScrewArray& screwAxes);
