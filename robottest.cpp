#include <iostream>

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "robotmath.h"

using Eigen::Vector3f;
using Eigen::Matrix3f;


void generalTest() {
    std::cout << "Make a new transformation matrix that is just the identity:\n";
    Affine3D myTransformation = Affine3D::Identity();
    std::cout << myTransformation.matrix() << '\n';

    std::cout << "Now we translate the matrix:\n";
    myTransformation.translate(Vector3f(1, 2, 3));
    std::cout << myTransformation.matrix() << '\n';

    Affine3D newTransformation = Affine3D::Identity();
    newTransformation.translate(Vector3f(4, 5, 6));

    myTransformation = newTransformation * myTransformation;
    std::cout << myTransformation.matrix() << '\n';

    // std::cout << "Rotate by some axis:\n";
    // float angle = 0.5 * EIGEN_PI;
    // Vector3f axis(1, 1, 1);
    // axis.normalize();

    // Twist screw;
    // screw << 0, 0, 1, 0, -2, 0;

    // Vector6f beans;
    // beans << 1, 2, 3, 4, 5, 6;
    
    // Twist meow = beans;
    // std::cout << meow << '\n';
    // std::cout << screw + beans << '\n';

    // Affine3D trans = matrixExpTransformation(0.5 * EIGEN_PI, screw);
    // std::cout << trans.matrix() << '\n';
}


void forwardKinematicsTest() {
    // define linkage lengths
    const int L1 = 4;
    const int L2 = 3;

    // set up zero configuration matrix
    Affine3D zeroConfiguration = Affine3D::Identity();
    zeroConfiguration.translate(Vector3f(L1 + L2, 0, 0));

    ScrewArray screwAxes = (
        ScrewArrayTranspose() << 
            0, 0, 1, 0, 0, 0,   // screw axis for joint 1
            0, 0, 1, 0, -L1, 0  // screw axis for joint 2
    ).finished().transpose();

    // theta_1 = pi/4, theta_2 = pi/4
    JointAngleVector vec_theta;
    vec_theta << 0.25 * EIGEN_PI, 0.25 * EIGEN_PI;
    
    Affine3D transformation = forwardKinematics(vec_theta, screwAxes);
    
    zeroConfiguration = transformation * zeroConfiguration;
    std::cout << zeroConfiguration.matrix() << '\n';
}


void forwardKinematicsUR5() {
    // define linkage lengths
    const int L1 = 3;
    const int L2 = 3;
    const int L3 = 3;
    const int L4 = 3;
    const int L5 = 3;
    const int L6 = 3;

    // set up zero configuration matrix
    Affine3D zeroConfiguration = Affine3D::Identity();
    zeroConfiguration.translate(Vector3f(L1 + L2 + L3 + L4 + L5 + L6, 0, 0));

    ScrewArray screwAxes = (
        ScrewArrayTranspose() <<
            1, 0, 0,    0, 0, 0,
            0, 0, 1,    0, -L1, 0,
            0, 0, 1,    0, -(L1 + L2), 0,
            0, 0, 1,    0, -(L1 + L2 + L3), 0,
            0, 1, 0,    0, 0, L1 + L2 + L3 + L4,
            1, 0, 0,    0, 0, 0
    ).finished().transpose();

    // all angles are pi/4
    JointAngleVector vec_theta = JointAngleVector::Constant(0.25 * EIGEN_PI);
    
    Affine3D transformation = forwardKinematics(vec_theta, screwAxes);
    
    Affine3D finalConfiguration = transformation * zeroConfiguration;
    std::cout << finalConfiguration.matrix() << '\n';
}


int main() {
    // generalTest();
    forwardKinematicsTest();
    // forwardKinematicsUR5();

}