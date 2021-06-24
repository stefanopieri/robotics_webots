# Braitenberg vehicle in WeBots

## Introduction
The aim of the project is to develop a Braitenberg vehicle that is able to  interact with an environment to achieve a goal. 
The project was developed in the WeBots environment.

## Overview
 The idea behind the project is to exploit a Braitenberg vehicle to find a treasure inside a haunted castle, represented by a Rectangle Arena. 

The goal of the robot is to find the treasure while avoiding obstacles and ghosts. 
The robot is developed to carry out 'fear' behaviour with both obstacles and ghosts and 'love' behaviour with the treasure.

The obstacles are represented by the borders of the Arena and by some boxes that are put inside the Arena. Ghosts are represented by static Light Sources with different intensity and radius. The treasure is a box whose color is gold (RGB palette = (255, 191, 0)) and whose model is 'treasure'. 

## Braitenberg Vehicle
The vehicle is a four wheeled robot that incorporates a total of 11 sensors:

● Two frontal light sensors
● Two frontal distance sensors
● Four lateral distance sensors, two for each side
● Two GPS sensors
● One Camera

The Light Sensors are used to frontally detect ghosts. 
The Distance Sensors (DS) are used to detect obstacles frontally and laterally. Also, the lateral DS pairs are used to stabilize the robot on a parallel trajectory when dealing with 'large' obstacles - such as in the case of the Arena walls. In this scenario, the trajectory is parallel up to a certain threshold value, which in practice can make the robot move slightly unparallel to the walls.

## The Arena


## Video Simulation


## References

## Purpose
This project was developed during the Robotics module of the MSc course in Artificial Intelligence offered by University of Huddersfield.
As previously stated, the project is not finished and it could contain bugs.
