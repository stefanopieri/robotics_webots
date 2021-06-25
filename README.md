# Braitenberg vehicle in WeBots

## Introduction
The aim of the project is to develop a Braitenberg vehicle that is able to achieve a certain goal while interacting with an environment.
The project was developed in the WeBots and the controller code is written in Python.

## Overview
The idea behind the project is to exploit a Braitenberg vehicle to simulate a treasure quest inside a haunted castle, which is represented by a Rectangle Arena. Some obstacles are put inside the castle and the robot is programmed to find the treasure while avoiding them. In terms of Braitenberg vehicles' basic behaviours, the robot is developed to act with 'fear' when it detects obstacles and with 'love' when it detects the treasure.

In the current implementation, there are two kind of obstacles: objects and ghosts.
Obstacles are represented by boxes and by the borders inside the Arena. Ghosts are represented by static Light Sources with different intensity and radius. 
The difference between objects and ghosts as obstacles is given by the way they are detected: objects are detected via Distance Sensors (DS), while ghosts via Light Sensors (LS).

The treasure is a box with some peculiarities - its color is gold (RGB palette = (255, 191, 0)) and its model is 'treasure'. 

## Braitenberg Vehicle

### Sensors Implemented 
The vehicle is a four wheeled robot that incorporates a total of 11 sensors:

- Two frontal LS
- Two frontal DS
- Four lateral DS, two for each side
- Two GPS sensors
- One Camera

LS are used to frontally detect ghosts. 
DS are used to detect obstacles frontally and laterally. Also, the lateral DS pairs are used to stabilize the robot on a parallel trajectory when dealing with 'large' obstacles - such as in the case of the Arena walls. In this scenario, the trajectory is parallel up to a certain threshold value. In practice, with the current threshold value robot can move almost parallel to the walls or large objects.

### Controller Code

... work in progess ...

#### 1. Steering Operations

... work in progress ...

#### 2. Obstacle and Treasure Detection
DS detect obstacles on both sides at near and very-near distance, making the robot respectively turn and rotate on the spot in the opposite direction. Ghost detection happens only frontally, so when the LS return a value higher than LS_THRESHOLD a ghost is detected.
Until the goal has not been spotted by the camera, the robot explores the space by moving forward and eventually avoiding obstacles and ghosts. The treasure is recognized when an object on camera makes the following conditions true:

- Treasure’s model is “treasure” AND  
- Treasure’s color is gold

Every time the goal is recognized, its position is saved in the GOAL_POSITION variable. Once the GOAL_POSITION is stored, the robot moves towards the goal. If the goal is detectable on camera, the goal’s coordinates are used to align the robot to the goal. Otherwise, the software calculates the equation of the line passing through the goal and the back GPS coordinates; then, it rotates the robot until the front GPS is aligned with the goal. In both cases, the front GPS must be closer than the back GPS to the treasure.

#### 3. Alignment with the Treasure

... work in progress ...

## The Arena

Here below the map used for tests

<img width="191" alt="castle_map" src="https://user-images.githubusercontent.com/49000357/123437242-bf569080-d5cf-11eb-9692-ff3a08087c15.png">


It is possible to notice that the robot usually takes two routes. They are called Route A and Route B

![route_images](https://user-images.githubusercontent.com/49000357/123437485-00e73b80-d5d0-11eb-9b42-9fbfa47c13d6.jpg)

## Analysis
The aim of this section is to provide a performance analysis of what is developed. In particular, a visual analysis of the impact of the model's parameters on the performance of the robot is proposed, with the idea of understanding how parameters impact the quality of the travel. To better understand the latter aspect, the simulation was launched after varying the value of one parameter at a time.
The performance metric used to assess the quality of the parameters set is the time in seconds to reach the treasure - also referred to as travelling time. 

### Alignment threshold
The ALIGNMENT_THRESHOLD parameter defines an acceptance limit for the robot-goal alignment.
It is used to set alignment in both cases if the treasure is framed and not framed by the camera. The necessary condition for alignment is to know the trasure's position and it is reasonable to consider this parameter more critical in situations in which the treasure is not framed.
Here below a graphics that depicts the travelling time vs smoothing factor.

<img width="542" alt="Schermata 2021-06-25 alle 16 42 46" src="https://user-images.githubusercontent.com/49000357/123441638-60dfe100-d5d4-11eb-8fdb-f3e85c48466a.png">

If ALIGNMENT_THRESHOLD < 0.05 and 0.2 ≤ ALIGNMENT_THRESHOLD ≤ 0.25, the robot finds the treasure and performances range from 25 to 29 s. For 0.05 < ALIGNMENT_THRESHOLD < 0.2, the time leaps by approximately 10 s due to the presence of an additional trajectory loop in the last part of the route.
Lastly, for ALIGNMENT_THRESHOLD ≥ 0.3, the robot fails to converge to the goal in less than 100 s. As one may expect, higher alignment tolerance implies a failure in convergence. However, it is unclear why travelling time is higher in the range 0.05 - 0.2.

### Smoothing factor
The smoothing factor is a parameter used to set wheels velocity, and in particular it is used to divide the maximum velocity in the
setVelocity method. In fact, the speed of the k-th wheel is set as following:\
wheel[k].setVelocity(MAX_VELOCITY / SMOOTHING_FACTOR)

In practice, the smoothing factor is used to control the wheel's velocity.
In fact, according to the previous formula, the higher the smoothing factor the lower the wheel velocity.
Here below the chart shows the travelling time vs smoothing factor

<img width="539" alt="Schermata 2021-06-25 alle 18 22 22" src="https://user-images.githubusercontent.com/49000357/123455675-4ad91d00-d5e2-11eb-850e-ab45b4eb859d.png">

The traveling time is lowest and constant for values in (0,1), because in such cases the resulting velocity exceeds the limit speed (MAX_VELOCITY set at 20) and the wheels speed is capped at 20. For values in [1, 4], traveling time grows linearly. For higher values, the chart shows oscillations around the trendline. As expected, the overall trend shows that time increases with the SMOOTHING_FACTOR, meaning that a lower wheel speed implies higher travelling times.

### LS threshold

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

Also, no research was made to find the optimal parameters for object and ghost detection; this aspect could contribute to the creation of loops.

Moreover, other sensors can be added to the current implementation and can help stabilizing the trajectory. 
Lastly, the current implementation focuses exclusively on controlling the interactions with the environment. An effective control on the actual behaviour is not present, as the control happens with an open loop and no feedbacks are used. Future development will focus on implementing more robust control strategies, such as closed loop / PID control strategies.

## References


https://github.com/KajalGada/Youtube-Tutorial-Download-Material




... work in progess ...

## Purpose
This project was developed during the Robotics module of the MSc course in Artificial Intelligence offered by University of Huddersfield.
As previously stated, the project is not finished and it could contain bugs.
