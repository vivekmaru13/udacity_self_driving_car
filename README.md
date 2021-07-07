# Udacity NanoDegree - Self Driving Car Engineer
Projects done under **[Udacity's Self Driving Car Engineer Nanodegree](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013)**.

The program has two main core modules as below.
1. Computer Vision, Deep Learning, and Sensor Fusion
2. Localization, Path Planning, Control, and System Integration

Each core module has it's own very specific projects that ensures that the knowledge conveyed in theory is indeed applicable to the real world scenarios. 

This repository contains all the projects that I have completed as part of this NanoDegree. The complexity of projects increases as you progress through the course.
I hope this repository/code can be helpful to someone who might need a helping hand on their journey.  

Please be aware that the Plagiarism policies are in place from Udacity. Please do not directly copy the code solutions, which might lead to disqualification.
Please read the Honour Code [here](https://udacity.zendesk.com/hc/en-us/articles/210667103-What-is-the-Udacity-Honor-Code-)

## Table of Contents

#### [P1 - Detecting Lane Lines](P1-LaneLines)
 - **Summary:** Detected highway lane lines on a video stream. The video data provided was captured during a drive with a windshield mounted camera. Used OpencV image analysis techniques to identify lines, including Hough Transforms and Canny edge detection.
 - **Key learnings:** Computer Vision
 
 #### [P2 - Advanced Lane Lines](P2-Advanced_Lane_Lines)
 - **Summary:** Built an advanced lane-finding algorithm using distortion correction, image rectification, color transforms, and gradient thresholding. Identified lane curvature and vehicle displacement. Overcame environmental challenges such as shadows and pavement changes.
 - **Key learnings:** Computer Vision
 
 #### [P3 - Traffic Sign Classifier](P3-Traffic_Sing_classifier)
 - **Summary:** Built and trained a CNN to classify traffic signs. Tested the model on random traffic signs from the internet and it performs well over 98%.
 - **Key learnings:** Machine Learning, CNN, Deep Learning
 
 #### [P4 - Behaviour Cloning](P4-Behaviour_Cloning)
 - **Summary:** Gathering data using the Udacity simulator and cloning the driving behaviour using an End-to-End learning model (CNN) from Nvidia. The trained model actually maneuvers the car successfully without leaving the track.
 - **Key learnings:** Machine Learning, CNN, End-to-End Learning, Deep Learning
 
 #### [P5 - Extended Kalman Filter](P5-Extended_Kalman_Filter)
 - **Summary:** This project deals with sensor fusion as the core topic. Implemented an extended Kalman Filter which tracks the velocity and position of a bicycle using simulated LiDar and Radar measurements.
 - **Key learnings:** C++, Kalman Filter, Sensor Fusion
 
 #### [P6 - Kidnapped Vehicle](P6-Kidnapped_Vehicle)
 - **Summary:** This project involves development of 2 dimensional particle filter which will help us to locate a kidnapped vehicle/robot. The filter uses a map and some localization information to determine the the new/final location of the vehicle/robot.
 - **Key learnings:** C++, Particle Filter, Sensor Fusion
 
 #### [P7 - Highway Driving](P7-Highway_Driving)
 - **Summary:** The main goal here is to create a path planning algorithm to generate smooth and safe trajectories for the vehicle in a highway setting. The highway scenario contains multiple vehicles driving at different speeds. Our car transmits and receives data regarding the location and based on that it generates trajectories. 
 - **Key learnings:** C++, Sensor Fusion, Path Planning
 
 #### [P8 - PID Control](P8-PID_Control)
 - **Summary:** Built a PID controller to maneuver the car on the track and adjust the steering angle to keep it on the track. 
 - **Key learnings:** C++, PID Controller
 
 #### [P9 - Capstone / System Integration](P9-Capstone)
 - **Summary:** This project incorporates all the learnings in this course together to design a system that can be deployed in a real car. Different car control and detection modules are integrated with each other using ROS. After finishing the integration part, the system can be tested on Udacity's simulator as well as a real Self-Driving Car, Carla.
 - **Key learnings:** C++, ROS