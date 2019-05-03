# Robot Arm Dynamics Library

This library provides an c++ class to calculate torque of robot joints using Newton-Euler method.

The class should be initialized with joint number, mass and mass center of each link, inertial matrix of links, and damping and friction parameters of each joint. All these data should be provided from base to tip in the form of std::vector with length equal to robot joint number.

In order to calculate joint torques, the needed parameters are: rotation matrices (or quaternions), translation vectors, joint rotation velocity and joint acceleration. Gravitational acceleration is also needed to compensate the gravity.

## Required packages

Eigen
