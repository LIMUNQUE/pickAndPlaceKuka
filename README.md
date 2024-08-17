# vision-robot-arm

Continuation of the [Vision-Robot-Arm Project:](https://github.com/LIMUNQUE/vision-robot-arm/tree/master) The purpose of this project is to create a pick-and-place color classifier using artificial vision with OpenCV. This robotic arm (ARM) was developed in ROS Noetic as the final assignment for the Serial and Mobile Robots course. We’ve chosen the KUKA KR3 model design for lightweight tasks.

For a quick test, you can use the following instructions:

```sh
roslaunch kr3_gazebo kr3_gazebo.launch gripper_2f:=true camera:=true
```
```sh
roslaunch kuka_arm_2f_moveit move_group.launch
```
```sh
rosrun camera_processing image_processor.py
```
```sh
rosrun kr3_master traj_coor.py
```


<picture> <img src="https://i.ibb.co/SKtzZnr/Screenshot-2024-08-10-211803.png" width = 500px></picture>
