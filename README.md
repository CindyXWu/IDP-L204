# IDP-L204

Code for 2nd year group robotics project. Goal: simulate in Webots environment a closed arena with randomly generated blocks. 
Half have a red LED on top, and half have a green LED on top.
Start two robots off in starting squares. Their goal is each to find blocks assigned to their colour.
We designed the robot sensor placement and arm mechanism for collecting blocks and modelled it in Webots.
This repository contains code for navigation, GPS communication, block detection, collision avoidance and debugging edge cases such that all blocks of the correct colour are
placed in the relevant starting square on the board within 3 minutes.
This was done first in C++ (code mostly complete, but buggy) and Python (fully functioning).
