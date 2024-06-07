# Swarm Robotics 

## Introduction 

This is the Code base of the Project Swarm Robotics of Electronics club. This is a Summer Project that has been undertaken in the Summer of 2024.

### Project Overview 

The Project involved developing an infrastructure of Swarm Robots that can be used in several use cases with minimal modifications in their Design and Circuitry. 
We are designing a Multi-Terrain Robot that can maneuver in any terrain and gather critical information. 
**Developing an in-house PCB
**Developing an IMU based localization system
**Developing a system of connecting various such small robots to a mother bot.

## WEEK 0

In week 0, basics of python and python assignment is done. Also, git and github is covered. 
We’re using Git and GitHub as tools for version control, collaboration and project management, thus ensuring everyone can work together efficiently.  

## WEEK 1

### Microcap
In week 1, we started with the microcap. Micro-Cap is used for the analysis and simulation of various circuits. Multiple circuits are analyzed for familiarity with Micro-Cap. Simulation of the circuit where the transistor works as a switch was done on Micro-Cap for verification and to ensure proper working.
### Solidworks - Assignment I
SolidWorks is instrumental in building the Swarm Robot by enabling detailed 3D modeling and design of mechanical components, ensuring precise fits and functionality. 
It allows for comprehensive simulation and analysis of movement, stress, and performance, helping to identify and resolve potential issues early.

##  WEEK 2

### KiCad
KiCad is a powerful and open-source EDA software, used for designing circuit schematics and PCBs. It is used to create detailed schematics that include all components, such as micro-controllers, sensors, actuators, and communication modules.
###  Solidworks - Assignment II
SolidWorks is being used for the rough designing of the 3D model of our car. It assists in analysing weight and design of the swram robotics model efficiently.
This reduces errors and optimizes the design before actual prototyping and manufacturing, saving time and resources.

## WEEK 3

### KALMAN FILTER
Kalman filtering (KF) is an optimal estimation algorithm that provides estimates of some unknown variables given the measurements observed over time.
The gyroscope provides the angular velocity and accelerometers provide linear acceleration . We then combine these two data to improve accuracy by implementing KF.
#### GNSS-INS Stimulation
GNSS-INS-SIM is an GNSS/INS simulation project, which generates reference trajectories, IMU sensor output, GPS output, odometer output and magnetometer output.
It allows users to model and test their algorithms in different scenarios, providing a flexible platform for developing and validating navigation solutions without needing extensive field tests.
### PCB Design
In our project, we use KiCad to design PCBs that integrate motor driver ICs (e.g., L298) with microcontrollers and other components. In robotics we require microcontrollers to process data and control actuators. Using KiCad, we can design custom microcontroller boards (Arduino nano 33 IoT) that meet specific requirements. We can make our own symbols and can add appropriate footprints to them.
### ROS and Controls
ROS (Robot Operating System) is an open-source, flexible framework for writing robot software. The project involves the coordination of multiple robots to perform tasks in a decentralized manner. A robot in the swarm operates based on local information and interactions with its neighbors.

Resources : 
ETH Zurich’s Programming for Robotics - ROS
Robotics Back End - ROS Noetic For Beginners

## WEEK 4

### PCB Design - BMS
Motors generally require more current than the Arduino can safely provide. So we are going to use an external power source for motors. We will be using BMS for providing required power to arduino as well as motors and use transistor (NMOS) as a switch.
