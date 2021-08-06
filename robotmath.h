#pragma once

#include "Eigen/Dense"
#include "Eigen/Geometry"


typedef Eigen::Transform<float, 3, Eigen::TransformTraits::Affine> Affine3D;


// returns the skew-symmetric representation [v] of a vector v in R3.
// this is defined such that for another vector w, [v]w = v x w
// where x denotes the cross product.
Eigen::Matrix3f skewSymRepr(const Eigen::Vector3f& vector);


// axis must be normalised!
// Eigen already provides "functionality" to do this: this is "equivalent" to
// ```
// Affine3D transformation = Affine3D::Identity();
// transformation.rotate(Eigen::AngleAxisf(angle, axis));
// ```
Eigen::Matrix3f matrixExpRotation(float angle, const Eigen::Vector3f& axis);


class Screw : public Eigen::Matrix<float, 6, 1> {
    public:
        
};