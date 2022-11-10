# ROS_handy_joint_angles

## Overview
The ROS_handy_joint_angles is part of a multi-repository project to create a life-size, working, Mr. Handy Robot from the Fallout Game series. 
The goal of this project is to have three indivdual 3DOF arms mounted to a central slew drive. Each arm will be capable of both Forward and Inverse Kinematics, will have predetermined locations, and each will be equiped with a different end-effector. 

The Mr. Handy will be able to:
- Choose an arm based on the action/ end-effector needed
- Move to the desired locations or predetermined locations
- Respond using audio files from the game files 

## Hardware Setup
This setup uses:
- Main Compute Module: Small Desktop Computer
  - Ιntel Celeron J4125 Quad Core Processor
  - 8GB RAM
  - 128GB SSD 
- Motor Controller: 
  - Arduino UNO 
  - PCA9685 16 Channel, 12 Bit PWM Servo Driver Board
- Motors: 
  - MG995 Servo Motor (20KG)
  
## This Repo
This repository is a ROS Package for the poject. This package includes custom nodes for: 
- Gathering input from a remote device (pipboy/nearby terminal)
- Chosing an arm with the applicable end-effector
- Computing joint angles given a target for the end-effector
- Sending joint angles to the servo-control board



