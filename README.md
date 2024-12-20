# homework4
# RL Fra2mo Description

This repository contains the ROS 2 packages for simulating and controlling the `rl_fra2mo` robot, including navigation, exploration, visualization, and marker detection. Follow the instructions below to set up and use the project.

## Prerequisites
Ensure that you have a ROS 2 workspace set up and ROS 2 installed on your system. Clone this repository within the `src` directory of your ROS 2 workspace.

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/your-repo-name/rl_fra2mo_description.git

2. Build the workspace:
    ``` bash
    colcon build 

3. Source the workspace:
    ```bash
    source install/setup.bash

## Launching the Simulation

To launch the Gazebo simulation:

```
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py marker_id:=115 marker_size:=0.2 reference_frame:=camera_link_optical
```

## Exploration and navigation

To start the exploration and navigation:

```
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py
```

## Visualization 

To visualize the exploration in RViz + rqt_image_view:

```
ros2 launch rl_fra2mo_description display_fra2mo.launch.py 
```

To eventually change the desired RViz configuration, it is possible to select the file through the commands:

1. 
    ```
    ros2 launch rl_fra2mo_description display_fra2mo.launch.py file:=slam_view.rviz
    ```
2.  
    ```
    ros2 launch rl_fra2mo_description display_fra2mo.launch.py file:=navigation.rviz
    ```
3. 
    ```
    ros2 launch rl_fra2mo_description display_fra2mo.launch.py file:=base
    _conf.rviz
    ```

## Robot Behaviors

### Following specific goals

Make the robot follow four predefined goals:
```
ros2 run rl_fra2mo_description follow_waypoints.py --mode 4goals
```

### Exploring the entire map

Allow the robot to explore the whole map:

```
ros2 run rl_fra2mo_description follow_waypoints.py --mode scan
```

### Testing exploration parameters

Test the exploration parameters for tuning:
```
ros2 run rl_fra2mo_description follow_waypoints.py --mode test_explore
```

### Approaching Obstacle 9

Make the robot approach obstacle 9 to detect the ArUco marker:

```
ros2 run rl_fra2mo_description follow_waypoints.py --mode obstacle_09
```

To publish the pose of the ArUco marker as a tf, with respect to the map frame,
```
ros2 run rl_fra2mo_description aruco_map_tf
```
