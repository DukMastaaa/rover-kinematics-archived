# rover-kinematics-archived (archived)

This is an abandoned repository containing math and algorithms used for
rigid body dynamics in the context of forward/inverse kinematics.
It uses the [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) C++ linear algebra 
library as a dependency, and formulas are taken from the textbook "Modern Robotics: Mechanics, 
Planning, and Control" by Kevin M. Lynch and Frank C. Park.

Currently, forward kinematics is working, but I am not sure why inverse
kinematics isn't. If anyone wants to work on this in the future, I suspect that
the `Eigen::CompleteOrthogonalDecomposition` thing I'm using for least squares
isn't working how I think it is --- see the `inverseKinematics` function
in `robotmath.cpp`.

The `build.bat` file is a really primitive "makefile", you can compile it in a similar way.
