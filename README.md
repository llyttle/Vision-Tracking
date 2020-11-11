# Vision-Tracking
Comprobo Vision Tracking Project

[insert image of neato soccer at work]

Here we see a robot using vision processing to kick and score goals. Read on to understand what is happening.

##Intro:
This is a project for Olin College of engineering's 2020 fall semester course Computational Robotics. The authors of this writeup and implementation are:

Timothy Novak

Loren Lyttle

The goal of this project was generally to develop a robotic system which reacts to visual input taken through a camera. The more specific goal of this project was to attempt to use visual processing to create a robot which could kick a ball towards a goal.

##Table of Contents
Overview
Our Approach
    the controller
    object identification
Design decisions
Challenges
Project extensions
lessons learned

## Overview:

Computer vision, or the process of a computer using a camera to percieve its environment, is an important skill to understand when designing robotic systems. This is because cameras are generally a cheap information dense source of information. This makes them an ideal canidate for a sensor on a robotic platform. However, computers unlike our minds can't readily process images and extract useful information and so many techniques have been developed to allow computers to process images. These range from complex machine learning algorithms such as you would see on a self driving car, or more simple image adjustments such as color balance corection which is available on some drones.

Due to the importance of computer vision for robotics projects we designed our project to provide a good environment for learning fundamental computer vision concepts and integrating them into a robotic controll scheme. The project we setteled on was robotic soccer where the robot would have to use a camera to find a ball and a goal and manuver itself in order to kick the ball into the goal. This would help us learn some of the fundamental computer vision tools such as object detection, object localization in an image, image filtering, and camera distortion then we could take those concepts and actually implement them in a useable way to inform a controll algorithm for the robot's movement.

## Our Approach

### The controller

The controller has a few fundamental behaviors it switches between to create the behavior of the robot.

#### Searching

The default behavior of the controller is that it tries to locate the ball and the goal in the global cordinate frame. The main action the robot will make during this process is to turn while looking for the ball. This way the robot can locate the ball even if it is not in the camera's line of sight.

[TODO: gif of bot looking for ball]

The robot also starts with knowlege of where it is located in the map and so it keeps an internal notion of the location of the goals so it can locate them even if they are outside of the camera frame. If the robot sees a goal, it will use that information to update its estimated position in the map.

'
#### Positioning

Once the ball was located, the neato needed to calculate where to go in order to bump the ball towards the goal. We decided to do these calculations in the map frame. Given the ball and odom_data, we created a transformer function to convert a poler coordinate in the base_link frame to a cartesian in the map frame. We also knew the position of each goal in the map, and decided that the center of each goal's opening would be the ideal spot to aim for. Looking at the image below, we used the vectors to the ball and goal from the robot to create a vector from the goal to the ball. This vector represented the direction the ball needed to travel to make a goal (albiet in the opposite direction). Extending this vector along it's trajectory allowed us to find the best position for the neato to be.

![normal dist](media/Neato_position.jpg)
<img src="Neato_position.jpg" width=300 />

## Challenges

#### Losing sight

One of the largest drawbacks of using camera vision rather than LIDAR is its limited viewing angle. When looking for the ball and turning the robot, the ball often skipped in and out of frame before it was recognized by the color filter. An effective solution to this problem was utilizing variable speed control for turning the robot. More specifically, the angular speed of the robot was proportional to how centered the ball was in the image. Slowing the rotation as the ball reached the center both eliminated the skipping issue and resulted in faster centering.

Another problem caused by the narrow window of the camera was keeping track of the ball when positioning the robot to kick it. Lining the neato up with the ball and the goal often required turning the neato away from the ball to head towards the more strategic position. However, many parts of our code relied upon knowing the ball's position and distance from the robot. Initially, this problem lead to the neato circling the ball, like it was too scared to leave it. Our state oriented Arbiter reduced this problem by ignoring these reliant functions until the neato made it to the lineup position.

#### Incorporation with LIDAR

Over the course of this project, we learned that both camera vision and LIDAR have their benefits and drawbacks. While the camera has a relatively narrow range of sight, the LIDAR has limited distance. One of the challenges in this project was combining the two to cover up for the other's weaknesses. For most of the project, the ball was considered 'defined' as long as it was in view of the camera. Being 'defined' triggered other functions, such as calculations for the best angle to kick it. Later we realized that, while a ball at the far end of the soccer field was easily visible to the camera, the LIDAR registered 'inf' untill the distance was under ten meters. Our solution to this was to further specify the conditions under which a function would be called. Defining these parameters was a meticulous and time consuming process.

## Future Improvements

#### Ball Path Estimation

One of the drawbacks of our current model is that it doesn't work well for kicking moving objects. The robot sees the ball once, calculates how to kick it to the goal, then moves to that point before checking the position of the ball again. If the ball were moving fast, the neato would be constantly behind. An interesting direction for further exploration may go into predicting where the ball will be by the time the neato can make it there. Real soccer players must do this constantly, and barely even think about predicting the uninterupted path of the ball. However this method may be too much for the current gazebo setup we are using. It was not clear in our study whether the refresh rate of the ball's position was fast or accurate enough to compute ball velocities and directions while the neato itself is moving. Perhaps better combining the LIDAR and camera, or integrating a new sensor would make this more feasible.

#### Defence Neato's

In soccer offense is not everything. To play a real game with neatos (at least the simplest 1 versus 1) the robot should know how to play defence. One way to do this would be to estimate the path of the ball, as previously mentioned. However a more sophisticated approach may use probability fields to determine the most likely position at which a rolling ball will be intercepted.  
