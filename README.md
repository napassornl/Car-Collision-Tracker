# Car-Collision-Tracker
This is a class assignment to predicting collisions of a large number of objects moving in a two-dimensional space.
The problem is relevant, for example, to air-traffic control, self-driving cars, or game simulations (asteroids, curling, pool).

# Physical Model
## Starting Scenario
My task was to designed C++ Algorithm to anticipate and model car object collisions given starting scenario of object ID, 
initial location, and velocity coordinates. The vehicles are specified in a file which gives an ID, a location at time t = 0
(x/y coordinates in meters) and velocity (x/y coordinates, in meters/second).

Here is an example of the file format:
2MU133 -34.94 -69.13 0.468 -0.900
0WI913 -43.08 92.12 -0.811 -0.958
6UP738 2.97 -66.25 -0.077 0.074
1IA244 72.94 -86.02 -0.665 -0.283
8RT773 -32.25 -2.63 -0.797 0.628
0HV350 -73.97 24.21 0.960 -0.870
0DU118 -82.09 44.95 0.661 -0.343
4FA522 -18.20 72.32 0.734 -0.990
1WR684 31.71 68.89 -0.509 -0.706

## Motion Model
The vehicles travel on a two-dimensional surface called the “field”. Each vehicle travels in a straight line at a constant velocity. The direction of travel is the direction of the velocity vector. Each vehicle will continue travelling in this direction forever, unless it collides with another vehicle.

## Collision Model
Two vehicles collide at the moment the distance between them becomes equal to the collision distance. For this problem, the collision distance is defined as 10 meters. When two vehicles collide, they are vaporized and are removed from the field. All other vehicles continue. My task was to discover all collisions, in the order that they occur, and determine the surviving vehicles. Time did not exist before t = 0, so if two vehicles would have collided in the past, we ignore this event. It never happened.

## My Output Format
The first line should specify how many vehicles there are.

Here is the correct output for the input file random10.coordinates:
there are 10 vehicles
collision report
at 18.0547 8RT773 collided with 0HV350
at 33.8258 4FA522 collided with 1WR684
the remaining vehicles are
2MU133 -34.94 -69.13 0.468 -0.9
0WI913 -43.08 92.12 -0.811 -0.958
6UP738 2.97 -66.25 -0.077 0.074
1IA244 72.94 -86.02 -0.665 -0.283
0DU118 -82.09 44.95 0.661 -0.343
7SW673 41.29 42.68 0.549 -0.012

