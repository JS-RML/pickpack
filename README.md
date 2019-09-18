# Pickpack

## 1. Overview

This package presents a complete software pipeline for picking thin,rigid objects. This perception-to-manipulation system for picking is fully autonomous system, incorporating visual perception module for obejct detection and localization with force/torque sensing module for environment perception. The manipulation module is the core component of the system, which is an implementation of **Tilt-and-Pivot** manipulaiton technique: a novel robotic object handling technique for picking thin objects lying on a flat surface through  robotic dexterous, in-handmanipulation. Picking thin objects is an important manipulation capability with a wide range of pratical applications such as bin picking tasks, product packaging tasks.  

This ROS package is directly applicable to an ordinary robotic setting featuring the conventional two- or three-fingered grippers installed on UR10 robot arm.

<p align = "center">
<img src="files/openlid.gif" width="360" height="202"> <img src="files/carton.gif" width="360" height="202"> 
<img src="files/acrylic_2finger.gif" width="360" height="202"> <img src="files/acrylic_3finger.gif" width="360" height="202">
</p>

**Video Link:** (https://www.youtube.com/watch?v=idWt6ZrfGDw)

**Authors**: Qianyi XU (qxuaj@connect.ust.hk), [Jungwon Seo](http://junseo.people.ust.hk/)  


## 2. Prerequisites

### 2.1 Hardware
1. [**Universal Robot UR10**](https://www.universal-robots.com/products/ur10-robot/)
2. [**Robotiq 140mm Adaptive parallel-jaw gripper**](https://robotiq.com/products/2f85-140-adaptive-robot-gripper)
3. [**Robotiq FT300 Force/Torque Sensor**](https://robotiq.com/products/ft-300-force-torque-sensor) 
4. [**Realsense Camera SR300**](https://www.intelrealsense.com/coded-light/)

### 2.2 Software
1. [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/)
2. [Driver for UR10 robot arms from universal robots](https://github.com/ThomasTimm/ur_modern_driver/)
3. [Universal Robot package for ROS Kinetic](http://wiki.ros.org/universal_robot)
4. [MoveIt!](https://moveit.ros.org/)
5. [Robotiq ROS package](http://wiki.ros.org/robotiq/)
6. [Mask R-CNN](https://github.com/matterport/Mask_RCNN)
7. [AprilTag ROS package](https://github.com/AprilRobotics/apriltag_ros)

```.ipynb```
files can be run in [jupyter notebook](https://jupyter.readthedocs.io/en/latest/install.html). Other requirements please check carefully in Mask R-CNN repository.

## 3. Build on ROS
In your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace):
```
cd ~/catkin_ws/src
git clone https://github.com/oliviaHKUST/pickpack.git
cd ..
catkin_make
```

## Maintenance 
For any technical issues, please contact: Qianyi XU (qxuaj@connect.ust.hk), Zhekai Tong (ztong@connect.ust.hk) and Tierui He (theae@connect.ust.hk)
