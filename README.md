# Robotics Project

MSCV Project in Robotics using ROS and Turtlebot Burger

# Introduction
## Description of the project
This report will describe in details the project given during Master in Computer Vision's 3rd semester in Robotics. In the given scenario we had to develop an autonomous driving robot. In general, a autonomous driving robot can analyzee surrounding without any human interactions and take decisions accordingly. In order to accomplish this implementation, a number of sensors are combing andd are used to identify the pathway and road signal from the surroundings.

## What is ROS?
The Robot Operating System (ROS) is an open source middleware which contains a set of libraries,softwares and tools that are used to facilitate the development of robotic applications. There is a plethora of features from sensor drivers to state-of-the-art algorithms. As middleware, it contains characteristics of both software and hardware, hencee, it is able to perform various actions like hardware abstraction and low level control.
Until now, different version of ROS exists with some crusial differences, so for compatibility reasons we are using the Melodic release.

## Robot used for this scenario
### Turtlebot Description
For the project, the mobile robot used is a Turtlebot3 Burger. The Turtlebot3 is a compact, modular and programmable mobile robot. It uses ROS and it is able to create multiple application for training research and development.

## Scenario Description
Our goal is to complete the following scenario:

1. **Camera Calibraton** is a crusial step for the fisheye camera which is integrated to Turtlebot3. The implementation uses the [camera_calibration](http://wiki.ros.org/camera_calibration) package from ROS. This packages uses OpenCV camera calibration, fully described [here](https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html). For this step we only have to use the checkerboard in order to get all the related coefficients for the undistortion.
![Checkerboard](img/checkerboard.jpg)
2. **Lane detection** is the keystone for the robot in order to be able to move according to the lines. In the race map that we used, the inner line is the yellow line and the outter line is the white one. To make the robot able to work we followed a "lane detection" calibration in which we set the most reliabe parameters of Hue Saturation and Value (HSV) to make the robot able to identify the `yellow` and `white` lines.
3. **Traffic Lights** is the last phase of the project. More specifically the turtlebot should be able to recognize 3 differert colors in order to get the right decision. Similarly, like the Lane Detection calibration, we adjust the aforementioned parameters for the `yellow`, `red` and `green` colours respectively.
# Repository Content
## Original Code links
The original code of this project is based on the tutorial of [Turtlebot_Autorace2020](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#turtlebot3-autorace-2020).
In this GitHub repository there is only the one of the misions (trafic light mission).Also, there are some additional file (images, calibration files, etc.).

## Package explanation and instructions
In this section, we briefly describe the packages and their dependencies that used for this project. [Here](https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#turtlebot3-autorace-2020) are the links that you can use in order to install all the related packages for the project.

### Package 1: traffic_light_camera
This package used for both extrinsic and intrinsic camera calibration. All the calibration steps are integrated into nodes. Also it contains a node which calls the `raspicam_node` which used in order to publish the camera frames. We will discuss below how it works.
#### Intrinsic camera calibration  
It uses the `camera_calibration` package. This package allows easy calibration of monocular or stero cameras. For this reason, we used the checkboard in order to fix the *Radial Distortion* of the acquired image. *Radial or Barrel Distortion* can be presented as:
$$
\begin{aligned}
x_{distorted}=x(1+k_{1}r^2+k_{2}r^4+k_{3}r^6)   
\end{aligned}
$$  
$$
\begin{aligned}
y_{distorted}=y(1+k_{1}r^2+k_{2}r^4+k_{3}r^6)
\end{aligned}
$$  
In the same manner, tangenial distortion occurs because the imaging-taking lense is not aligned perfectly parallel to the imaging plane. So, some images look nearer than expexted. The amount of tangenial distortion can be presented as below:  
$$
\begin{aligned}
x_{distorted}=x+[2*p_{1}xy+p_{2}(r^2+2x^2)]   
\end{aligned}
$$  
$$
\begin{aligned}
y_{distorted}=y+[p_{1}(r^2+2x^2)+2*p_{2}xy]  
\end{aligned}
$$  
According to the equation above, we can find the five paremeters, known as distortion coefficients
$$
DistortionCoefficients=(k_{1},k_{2},p_{1},p_{2},k_{3})
$$  
Furthermore, **intrinsic parameters** allows a mapping between camera coordinates and pixel coordinates in the image frame. They include information like local length $(f_{x},f_{y})$, and optical center $(C_{x}, C_{y})$. This parameters can be expressed in camera matrix:

$$
\begin{aligned}
camera matrix =
\begin{bmatrix}
   f_{x} & 0 & C_{x}\\
   0 & f_{y} & C_{y}\\
   0 & 0 & 1 
\end{bmatrix} 
\end{aligned}
$$ 
<!--  -->
#### Extrinsic camera calibration  
Extrinsic camera calibration defines a location and orientation of the camera with respect to the world frame. Similarly, we could state that they corresponds to rotation and translation vectors which translates a coordinates of a 3D point to a coordinate system.

**Image Projection** gets 4 coordinates of the image in order to get the projection according to these coordinates. 
<!-- TODO : Take a screenshot from the lab and add it as an example HERE -->
*image_compensation* node handle this using histogram equalization to improve the quality of the projected image.  


# Original Content
In this section, we will describe the original content we had the access in the project.
<!-- 
### Setup Material

### Packages

### Basic Commands

### Project's implementation

#### ROS Navigation

#### Move robot

#### Lane Detection

# Instruction to run the project

## Download and install 

## Building 

## Commands

# Result 

Show the nodes that are created for this project 

# Problems encountered

### On the turtlebot

### On the remote PC

# Conclusion

# References

## Credits
 -->
