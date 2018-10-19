# Thin_Obj_Bin_Picking
This is a ROS package presents a perception-to-manipulation system for picking thin objects from clutter. 
The manipulation consists of four steps: detecting the object of instance, approaching the target overhead, descending till the fingertip contacts the object, tilting to adjust the final grasp pose.
Object detection is implemented on Mask R-CNN, a deep neural network for instance segmentation, while descending and tilting are implemented with tactile sensors.

## Prerequisites

### Hardware
1. Universal Robot UR10
2. Robotiq 140mm Adaptive parallel-jaw gripper
3. RightHand Labs's TakkStrip
4. Arduino
5. Realsense SR300

### Software
1. [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/)
2. [Driver for UR10 robot arms from universal robots](https://github.com/ThomasTimm/ur_modern_driver/)
3. [Universal Robot package for ROS Kinetic](http://wiki.ros.org/universal_robot)
4. [MoveIt!](https://moveit.ros.org/)
5. [Robotiq ROS package](http://wiki.ros.org/robotiq/)
6. [Mask R-CNN](https://github.com/matterport/Mask_RCNN)

```.ipynb```
files can be run in [jupyter notebook](https://jupyter.readthedocs.io/en/latest/install.html). Other requirements please check carefully in Mask R-CNN repository.

## Get started
The following instructions will help you build up the software step by step.

1. Follow the tutorial in [Universal Robot package for ROS Kinetic](http://wiki.ros.org/universal_robot) and [Robotiq ROS package](http://wiki.ros.org/robotiq/) to set up hardware properly.
2. Run Realsense SR300 camera in ROS. See [link](http://wiki.ros.org/RealSense).
3. Connect Arduino and tactile sensor, output the sensor readings in ROS. See [link](http://wiki.ros.org/rosserial_arduino/Tutorials).
4. Setup frames:
   ```
   cd scripts
   ```
   ```
   python frame_transform.py 
   ```
5. Open a terminal, run object detection:
   ```
   cd samples
   ```
   ```
   jupyter notebook
   ```
   Open ```instance_segmentation.ipynb```. For loading ```BLISTER_MODEL_PATH```, please refer to [here](https://hkustconnect-my.sharepoint.com/:u:/g/personal/ztong_connect_ust_hk/EQ_7Mi8_-pBCrZwDXYc21QIBEgfSpwU2K-eZ1M3d01JVcQ?e=dr3e5G).
   
6. Open another terminal, run manipulation:
   ```
   cd scripts
   ```
   ```
   jupyter notebook
   ```
   Open ```thin_object_bin_pick_mani.ipynb```

## Author
Zhekai Tong (ztong@connect.ust.hk) and Tierui He (theae@connect.ust.hk)
