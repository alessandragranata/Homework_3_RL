# Homework_3_RL


## Insertion and Detection of a Circular Object into a Gazebo World

To construct a virtual world containing a blue circular object use the following instruction:

```bash

ros2 launch iiwa_bringup iiwa.spherical_object.launch.py use_sim:=true use_vision:=true

```

To implement a visual detection system using the vision_opencv package:

```bash

ros2 run ros2_opencv ros2_opencv_node

```

To visualize the detection run:

```bash

ros2 run rqt_image_view rqt_image_view

```

In the camera we will obtain the detection of the spherical blue object by selecting the `/random_image` topic.
 
## Aruco tag detection 

The detection results can be provided in this way:launch the iiwa.launch.py file including the true condition both for use_sim and use_vision which allow us to open the Gazebo world and to visualize via camera the aruco marker.

```bash

ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller" use_sim:=true use_vision:=true 

```      

To run the camera:

```bash

ros2 run rqt_image_view rqt_image_view 

```

and select the `/camera` topic.     

To detect the Aruco marker launch the single.launch.py file:

```bash

ros2 launch aruco_ros single.launch.py marker_size:=0.1 marker_id:=201

``` 

In the camera we will obtain the detection of the ArUco tag by selecting the `/aruco_single/result` topic.

## Running the ros2_kdl_vision_control.cpp

To run the executable file use the command:

```bash

ros2 run ros2_kdl_package ros2_kdl_vision_control

```

The arguments of `ros2_kdl_vision_control` are:

- `cmd_interface` (default: "position") - [position|velocity|effort].

- `trajectory_type` (default: "trapezoidal") - to choose the velocity profile [trapezoidal|cubic].

- `task`(default: "positioning") - to choose the task[positioning|look-at-point]
 
## Running Gazebo and Rviz

To spawn the robot in Gazebo and Rviz with the velocity controller:

```bash

ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true use_vision:=true command_interface:="velocity" robot_controller:="velocity_controller"

```

The arguments of `iiwa.launch.py` are:

- `use_sim` (default: "false") - Start robot in Gazebo simulation.

- `command_interface` (default: "position") - Robot command interface [position|velocity|effort].

- `robot_controller` (default: "iiwa_arm_controller" for the position_controller) - Robot controller to start.

- `use_vision` (default: "false") - Start camera in Gazebo simulation.
 
## IMPORTANT

When gazebo world is open, the simulation is in pause and with zero gravity. Then we click on the small play button in the bottom left corner of the GUI in order to start the simulation.
 
## Positioning Task

After opening the gazebo world, to visualize the aruco detection:

```bash

ros2 launch aruco_ros single.launch.py marker_id:=201 marker_size:=0.1

```

To visualize the Positioning task, it has to be run the ros2_kdl_vision_control file in this way:

```bash

ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args  -p cmd_interface:=velocity -p task:=positioning

```

## Look-at-point task

After opening the gazebo world, to visualize the aruco detection:

```bash

ros2 launch aruco_ros single.launch.py marker_id:=201 marker_size:=0.1 reference_frame:=camera_link_optical

```

To visualize the look-at-point task, it has to be run the ros2_kdl_vision_control file in this way:

```bash

ros2 run ros2_kdl_package ros2_kdl_vision_control --ros-args  -p cmd_interface:=velocity -p task:=look-at-point

```

 
