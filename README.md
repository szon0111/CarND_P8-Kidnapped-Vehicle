# Project 8: Kidnapped Vehicle
[//]: # (Image References)

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


Overview
---
Implement a 2 dimensional particle filter to localize a vehicle given some initial localization information, sensor data, and control data

#### The goals / steps of this project are the following:
* Localize the vehicle to within the desired accuracy 
* Make the particle run within the specified time of 100 seconds
* Test the code on the simulator

Project Deliverables
---
* `particle_filter.cpp` 

Results
---
View the **[video](https://youtu.be/SagWBrtoyvw)** on Youtube

x_error: 0.112
y_error: 0.109
yaw_error: 0.004

---

## How to run the simulator
* Download the simulator(link below) and open it. In the main menu screen select Project 3: Kidnapped Vehicle
* Run the executable ./particle_filter
* Once the scene is loaded click the START button to observe how the car drives and observes landmarks

## Dependencies

* Udacity Simulator [download](https://github.com/udacity/self-driving-car-sim/releases)
* uWebSocket [download](https://github.com/uWebSockets/uWebSockets)
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
