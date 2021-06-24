# Braitenberg vehicle in WeBots

## Introduction
The aim of the project is to develop a Braitenberg vehicle that is able to achieve a certain goal while interacting with an environment.
The project was developed in the WeBots and the controller code is written in Python.

## Overview
The idea behind the project is to exploit a Braitenberg vehicle to find a treasure inside a haunted castle, which is represented by a Rectangle Arena. Some obstacles are put inside the castle and the robot is programmed to find the treasure while avoiding them. In terms of Braitenberg vehicles' basic behaviours, the robot is developed to act with 'fear' when it detects obstacles and with 'love' when it detects the treasure.

In the current implementation, there are two kind of obstacles: objects and ghosts.
The obstacles are represented by some boxes that are put inside the Arena (and by its borders). Ghosts are represented by static Light Sources with different intensity and radius. The treasure is a box with some peculiarities - its color is gold (RGB palette = (255, 191, 0)) and its model is 'treasure'. 

The difference between objects and ghosts as obstacles is only given by how they are detected: objects are detected via Distance Sensors (DS), while ghosts via Light Sensors (LS).

## Braitenberg Vehicle
The vehicle is a four wheeled robot that incorporates a total of 11 sensors:

● Two frontal LS
● Two frontal DS
● Four lateral DS, two for each side
● Two GPS sensors
● One Camera

The Light Sensors are used to frontally detect ghosts. 
The Distance Sensors (DS) are used to detect obstacles frontally and laterally. Also, the lateral DS pairs are used to stabilize the robot on a parallel trajectory when dealing with 'large' obstacles - such as in the case of the Arena walls. In this scenario, the trajectory is parallel up to a certain threshold value. In practice, with the current threshold value robot can move almost parallel to the walls or large objects.

## The Arena
... work in progess ...

## Video Simulation
... work in progess ...

## References
https://github.com/KajalGada/Youtube-Tutorial-Download-Material
... work in progess ...

## Purpose
This project was developed during the Robotics module of the MSc course in Artificial Intelligence offered by University of Huddersfield.
As previously stated, the project is not finished and it could contain bugs.
