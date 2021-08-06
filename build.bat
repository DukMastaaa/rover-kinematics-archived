g++ -c -I ./Eigen/src robotmath.cpp -o robotmath
move robotmath ./bin/robotmath

g++ -c -I ./Eigen/src robottest.cpp -o robottest
move robottest ./bin/robottest

g++ -I ./Eigen/src ./bin/robottest ./bin/robotmath -o main