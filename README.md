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
