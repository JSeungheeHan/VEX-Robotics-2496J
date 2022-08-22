# VEX-Robotics-2496J
A collection of code used in 2019-2021 VEX Robotics team 2496J Robopatties.

## Library

The library.h file contains various functions used in the autonomous and driver code. It is compiled in C++, and uses the VEX Library to function. The turn functions takes the values from an inertia sensor to use the PID algorithm. This helps the robot make accurate turns. The drive forward functions uses the rotational sensors attached to the wheels. This helps us determine how far the robot has moved.

## Main

The main.cpp file contains the drive code and autonomous code used in VEX competitions. The drive code uses a driving mode called "tank drive", in which one joycon determines whether the robot moves forward or backwards, and the other determines whether it turns. This is different from "arcade drive", where each joycon corresponds to the set of left or right wheels.

The Autonomous code uses the functions listed in library.h in order to make the robot execute a series of actions at the beginning of the match. The game that the one posted was made for was Change Up, in which the robot would pick up balls from the field and score them into columns. This autonomous code scored balls in 5 of the 9 goals.
