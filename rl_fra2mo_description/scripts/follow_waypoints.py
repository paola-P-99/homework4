#!/usr/bin/env python3

import argparse
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import os
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import yaml
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import Twist
import time



def parse_arguments():
    parser = argparse.ArgumentParser(description="Navigation script with predefined YAML files and mode.")
    parser.add_argument("--mode", type=str, required=True, choices=["scan", "4goals", "test_explore", "obstacle_09"], help="Mode of operation: 'scan', '4goals', 'test_explore' or 'obstacle 9'.")
    return parser.parse_args()

def select_file(mode):
    files = {
        "4goals": "~/ros2_ws/src/rl_fra2mo_description/scripts/4goals.yaml",
        "scan_map": "~/ros2_ws/src/rl_fra2mo_description/scripts/scan_map.yaml",
        "test_explore": "~/ros2_ws/src/rl_fra2mo_description/scripts/test_explore.yaml",
        "obstacle_09": "~/ros2_ws/src/rl_fra2mo_description/scripts/obstacle_09.yaml",
    }
    return os.path.expanduser(files.get(mode, ""))

def load_waypoints(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)["waypoints"]

def transform_waypoints(waypoints_map):
    odom_origin = np.array([-3.0, 3.5, 0.1])
    odom_yaw = -1.57
    odom_rotation = R.from_euler('z', odom_yaw).as_quat()

    waypoints_odom = []
    for waypoint in waypoints_map:
        map_position = np.array([waypoint["position"]["x"], waypoint["position"]["y"], waypoint["position"]["z"]])
        translated_position = map_position - odom_origin
        rotated_position = R.from_quat(odom_rotation).inv().apply(translated_position)

        map_orientation = waypoint["orientation"]
        odom_orientation = transform_quaternion(map_orientation, R.from_quat(odom_rotation).inv().as_quat())

        waypoints_odom.append({
            "position": {
                "x": rotated_position[0],
                "y": rotated_position[1],
                "z": map_position[2],
            },
            "orientation": {
                "x": odom_orientation[0],
                "y": odom_orientation[1],
                "z": odom_orientation[2],
                "w": odom_orientation[3],
            }
        })
    return waypoints_odom

def transform_quaternion(quaternion, rotation_quaternion):
    q_map = R.from_quat([quaternion["x"], quaternion["y"], quaternion["z"], quaternion["w"]])
    q_transform = R.from_quat(rotation_quaternion)
    q_odom = q_transform * q_map
    return q_odom.as_quat()


class ArucoSubscriber(Node):
    def __init__(self):
        super().__init__("ArucoSubscriber")
        self.subscription = self.create_subscription(
            PoseStamped,
            "/aruco_single/pose",
            self.marker_callback,
            10)
        self.subscription
        self.marker_detected = False
    
    def marker_callback(self,msg):
        # Print the marker's pose
        self.marker_detected = True
        
        # print("Marker detected at:")
        # print(f"  Position: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}")
        # print(f"  Orientation: x={msg.pose.orientation.x}, y={msg.pose.orientation.y}, z={msg.pose.orientation.z}, w={msg.pose.orientation.w}")

def main():
    args = parse_arguments()
    rclpy.init()
    #aruco_subscriber = ArucoSubscriber()

    if args.mode == "scan":
        print("Performing scan operation...")
        # Load all 9 goals and execute them sequentially
        file_path = select_file("scan_map")
    elif args.mode == "4goals":
        print("Executing 4 goals in specific order...")
        # Load 4 goals and execute them in the specific order (goal3, goal4, goal2, goal1)
        file_path = select_file("4goals")
    elif args.mode == "test_explore":
        print("Executing goals in specific order...")
        file_path = select_file("test_explore")
    elif args.mode == "obstacle_09":
        print("Excuting goal to reach obstacle 9 and detect the Aruco marker...")
        file_path = select_file("obstacle_09")
    else:    
        print("Invalid mode.")
        return

    if not file_path:
        print("Invalid mode or no file associated with the mode.")
        return

    waypoints_map = load_waypoints(file_path)
    waypoints_odom = transform_waypoints(waypoints_map)

    navigator = BasicNavigator()

    def create_pose(transform):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = transform["position"]["x"]
        pose.pose.position.y = transform["position"]["y"]
        pose.pose.position.z = transform["position"]["z"]
        pose.pose.orientation.x = transform["orientation"]["x"]
        pose.pose.orientation.y = transform["orientation"]["y"]
        pose.pose.orientation.z = transform["orientation"]["z"]
        pose.pose.orientation.w = transform["orientation"]["w"]
        return pose

    all_goal_poses = list(map(create_pose, waypoints_odom))

    if args.mode in ["4goals","scan","test_explore"]:

        if args.mode == "4goals":
            # Esegui i 4 goals nell'ordine specificato: goal3, goal4, goal2, goal1
            execution_order = [2, 3, 1, 0]  # goal3, goal4, goal2, goal1
            goal_poses = [all_goal_poses[i] for i in execution_order]
        else:
        # Esegui tutti i goals nell'ordine sequenziale
            goal_poses = all_goal_poses

    else:

        goal_poses = [all_goal_poses[0]]
        init_position = [all_goal_poses[1]]


    navigator.waitUntilNav2Active(localizer="smoother_server")

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback and i % 10 == 0:
            print('Executing current waypoint: ' +
            str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()
            if now - nav_start > Duration(seconds=600):
                navigator.cancelTask()
        i += 1

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')
        
       
    if args.mode == "obstacle_09":
        
        aruco_subscriber = ArucoSubscriber()
        print("Waiting for Aruco marker to be detected...")

        # Pause to allow detection
        pause_duration = 10  # Pause duration in seconds
        print(f"Pausing for {pause_duration} seconds to detect the Aruco marker...")
        start_time = time.time()
        while time.time() - start_time < pause_duration:
            rclpy.spin_once(aruco_subscriber, timeout_sec=0.1)  # Keep spinning for callbacks

        if aruco_subscriber.marker_detected:
            print("Aruco Marker Detected!")
        else:
            print("Aruco Marker not detected within the pause duration.")   
        
        print("Returning to initial position...")  

        navigator.followWaypoints(init_position)  

        i = 0
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback and i % 10 == 0:
                print('Executing current waypoint: ' +
                    str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
                now = navigator.get_clock().now()
                if now - nav_start > Duration(seconds=600):
                    navigator.cancelTask()
            i += 1

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

    exit(0)

if __name__ == '__main__':
    main()
