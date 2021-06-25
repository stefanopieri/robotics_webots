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

### Sensors Implemented 
The vehicle is a four wheeled robot that incorporates a total of 11 sensors:

● Two frontal LS\
● Two frontal DS\
● Four lateral DS, two for each side\
● Two GPS sensors\
● One Camera

The Light Sensors are used to frontally detect ghosts. 
The Distance Sensors (DS) are used to detect obstacles frontally and laterally. Also, the lateral DS pairs are used to stabilize the robot on a parallel trajectory when dealing with 'large' obstacles - such as in the case of the Arena walls. In this scenario, the trajectory is parallel up to a certain threshold value. In practice, with the current threshold value robot can move almost parallel to the walls or large objects.

### Controller Code

... work in progess ...

Distance sensors detect obstacles on both sides at near and very-near distance, making the robot respectively turn and rotate on the spot in the other direction. Ghost detection happens frontally via Light Sensors. When the sensors return a value higher than a LS_THRESHOLD, a ghost is detected.
Until the goal has not been spotted by the camera, the robot explores the space by moving forward and eventually avoiding obstacles and ghosts. The goal is recognized when an object on camera makes the following conditions true:

● Treasure’s model is “treasure” AND\  
● Treasure’s color is gold

Every time the goal is recognized, its position is saved in the GOAL_POSITION variable. Once the GOAL_POSITION is stored, the robot moves towards the goal. If the goal is detectable on camera, the goal’s coordinates are used to align the robot to the goal. Otherwise, the software calculates the equation of the line passing through the goal and the back GPS coordinates; then, it rotates the robot until the front GPS is aligned with the goal. In both cases, the front GPS must be closer than the back GPS to the treasure.

## The Arena

... work in progess ...

## Video Simulation





https://user-images.githubusercontent.com/49000357/123314465-edce6000-d52a-11eb-9e40-6f09225fe8cf.mp4





... work in progess ...

## Conclusions
### Achievements
The current implementation is able to successfully interact with a fairly complex environment and is able to achieve the desired goal. 
One of the aims of the project is to explore the ability to interact with different kinds of obstacles and the current implementation shows acceptable results. As one may expect, it was shown that the robot is able to converge to the treasure only if the model’s parameters do not take 'pathological' values. Also, it is shown that using a 'good' set of parameters can significantly decrease the travelling time. 

### Future Improvements
Several limits of this project are known. Many aspects can be improved and will be object of future development. The camera is set to recognize only one object at a time and it recognizes only the nearest object. Significant improvements should be achieved once the ability to recognize all the framed object is implemented. 

Space exploration without knowing the tresure's position is not optimized and can lead to trajectory loops, as the current control code makes the robot go straight. Such an approach can lead to issues when dealing with more complex environments.
Two ideas are considered to improve on this aspect: 

- save past trajectories in memory and vary the current trajectory in order not to explore the same areas two times
- insert random variations in the trajectory

Also, no research was made to find the optimal parameters for object and ghost detection; this aspect could concur to creating trajectory loops.

Moreover, other sensors can be added to the current implementation and can help stabilizing the trajectory. 
Lastly, the current implementation focuses exclusively on controlling the interactions with the environment. An effective control on the actual behaviour is not present, as the control happens with an open loop and no feedbacks are used. Future development will focus on implementing more robust control strategies, such as in the case of closed loop / PID systems.

## References
https://github.com/KajalGada/Youtube-Tutorial-Download-Material

... work in progess ...

## Purpose
This project was developed during the Robotics module of the MSc course in Artificial Intelligence offered by University of Huddersfield.
As previously stated, the project is not finished and it could contain bugs.
