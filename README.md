This project is a part of exercises of the Udemy Course: [ROS For Beginners: Basics, Motion and OpenCV](https://www.udemy.com/course/ros-essentials/) by Anis Koubaa


##Objective: 
The goal of this project is to implement computer vision techniques to detect a tennis ball in a video stream, using ROS and OpenCV (Python)

##Code_Overview:
A publisher node continuously reads a video stream frame by frame using opencv functions and converting it to ros compatible format using CV_bridge.
The subcriber subscribes to the publisher topic and catches the message frame by frame, converting it to opencv compatible message and then, using opencv functions it tracks tennis ball
