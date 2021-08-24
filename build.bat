g++ -c -I ./Eigen/src -O1 -Wall -Wpedantic robotmath.cpp -o robotmath
move robotmath ./bin/robotmath

g++ -c -I ./Eigen/src -Wall -Wpedantic robottest.cpp -o robottest
move robottest ./bin/robottest

g++ -I ./Eigen/src -Wall -Wpedantic ./bin/robottest ./bin/robotmath -o main