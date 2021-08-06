#pragma once

#include "Eigen/Dense"
#include "Eigen/Geometry"

typedef Eigen::Matrix<float, 6, 1> Vector6f;  // technically not a Vector6f but close enough
typedef Eigen::Transform<float, 3, Eigen::TransformTraits::Affine> Affine3D;


// returns the skew-symmetric representation [v] of a vector v in R3.
// this is defined such that for another vector w, [v]w = v x w
// where x denotes the cross product.
Eigen::Matrix3f skewSymRepr(const Eigen::Vector3f& vector);


// calculates the rotation matrix (through matrix exponential) obtained
// by rotating about `axis` by `angle`. axis must be normalised!
// Eigen already provides "functionality" to do this: this is "equivalent" to
// ```
// Affine3D transformation = Affine3D::Identity();
// transformation.rotate(Eigen::AngleAxisf(angle, axis));
// ```
Eigen::Matrix3f matrixExpRotation(float angle, const Eigen::Vector3f& axis);


// returns the adjoint representation of a transformation matrix
// Eigen::Matrix<float, 6, 6> adjointRepr(const Affine3D& transformation);


// represents twists V = [w; v] and screws S = [Sw; Sv] in R6.
class Twist : public Eigen::Matrix<float, 6, 1> {
    public:
        // first three elements
        Eigen::VectorBlock<Vector6f, 3> angularPart();
        const Eigen::VectorBlock<const Vector6f, 3> angularPart() const;

        // last three elements
        Eigen::VectorBlock<Vector6f, 3> linearPart();
        const Eigen::VectorBlock<const Vector6f, 3> linearPart() const;
};


// returns the transformation matrix (through matrix exponential) obtained
// by following `screw` by `angle`. note `Twist` represents both twists and screws.
Affine3D matrixExpTwist(float angle, const Twist& screw);
