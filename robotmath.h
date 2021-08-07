#pragma once

#include "Eigen/Dense"
#include "Eigen/Geometry"

typedef Eigen::Matrix<float, 6, 1> Vector6f;  // technically not a Vector6f but close enough
typedef Eigen::Transform<float, 3, Eigen::TransformTraits::Affine> Affine3D;

constexpr const int jointCount = 2;  // change if required.

typedef Eigen::Matrix<float, jointCount, 1> JointAngleVector;

// This is intended to package screw axes into a single matrix for easy storage.
typedef Eigen::Matrix<float, 6, jointCount> ScrewArray;
typedef Eigen::Matrix<float, jointCount, 6> ScrewArrayTranspose;


typedef Eigen::Matrix<float, 6, jointCount> Jacobian;


// Returns the skew-symmetric representation [v] of a vector v in R3.
// This is defined such that for another vector w, [v]w = v x w
// where x denotes the cross product.
Eigen::Matrix3f skewSymRepr(const Eigen::Vector3f& vector);


// Calculates the rotation matrix (through matrix exponential) obtained
// by rotating about `axis` by `angle`.
// Warning: `axis` must be normalised!
// Eigen already provides "functionality" to do this: this is "equivalent" to
// ```
// Affine3D transformation = Affine3D::Identity();
// transformation.rotate(Eigen::AngleAxisf(angle, axis));
// ```
Eigen::Matrix3f matrixExpRotation(float angle, const Eigen::Vector3f& axis);


// Returns the adjoint representation of a transformation matrix
// Eigen::Matrix<float, 6, 6> adjointRepr(const Affine3D& transformation);


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


// Returns the transformation matrix (through matrix exponential) obtained
// by following `screw` by `angle`.
Affine3D matrixExpScrew(float angle, const Twist& screw);


// Returns the product of exponentials e^{ [S1]theta1 } e^{ [S2]theta2 } ...
// which can readily be multiplied by some initial configuration matrix M.
// Note that if screws are expressed in the body frame, post-multiply M 
// by the output. If screws are expressed in the space frame, pre-multiply
// M by the output.
Affine3D forwardKinematics(const JointAngleVector& vec_theta, const ScrewArray& screwAxes);