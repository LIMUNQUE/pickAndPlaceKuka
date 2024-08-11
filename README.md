# vision-robot-arm

Continuation of project  [Vision-robot-arm](https://github.com/LIMUNQUE/vision-robot-arm/tree/master)
The purpose of this project is to create a pick and place color classifier, using artificial vision from opencv.
This ARM was done in ROS Noetic as the final assignment for the serial and mobile robots course 

We are using the kuka kr3 model design for small duty

for a short test you can use:

```sh
roslaunch kr3_gazebo kr3_gazebo.launch gripper_2f:=true camera:=true
```

```sh
rosrun camera_processing image_processor.py
```



<picture> <img src="https://i.ibb.co/SKtzZnr/Screenshot-2024-08-10-211803.png" width = 500px></picture>
