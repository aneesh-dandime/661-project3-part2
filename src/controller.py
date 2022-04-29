#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from typing import List, Tuple
import tf
import numpy as np

class Robot():
    def __init__(self, rpm1, rpm2, clearance):
        self.RPM1, self.RPM2 = rpm1, rpm2
        self.clearence =  clearance
        self.radius, self.wheelDistance, self.dt = 0.038, 0.354, 0.1 

def make_path_messages(path_points: List[List[float]]) -> Path:
    path = Path()
    path.header.frame_id = "/map"
    for point in path_points:
        pose = PoseStamped()
        pose.pose.position.x = float(point[0])
        pose.pose.position.y = float(point[1])
        pose.pose.position.z = 0
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        path.poses.append(pose)
    return path

def compute_velocities(turtlebot_model: Robot, action: List[float]) -> Tuple[float, float]:
    r, l, _ = turtlebot_model.radius, turtlebot_model.wheelDistance, turtlebot_model.dt 
    ul = float(action[0])
    ur = float(action[1])
    linear_velocity = (r / 2) * (ul + ur)
    angular_velocity = (r / l) * (ur - ul)
    return angular_velocity, linear_velocity

def make_velocity_messages(robot: Robot, action: List[float]) -> Twist:
    vel = Twist()
    angular_velocity, linear_velocity = compute_velocities(robot, action)
    vel.linear.x = linear_velocity
    vel.linear.y = 0
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = angular_velocity
    return vel

def find_closest_point(pose, points: List[float]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    current_point = np.array([pose.position.x, pose.position.y]).reshape(-1,2)
    points = np.array(points, dtype = float).reshape(-1, 2)
    difference = points - current_point
    ssd = np.sum(np.square(difference), axis = 1) 
    idx = np.argmin(ssd)
    min_distance = np.sqrt(ssd[idx])
    delx, _ = difference[idx, 0], difference[idx, 1]
    min_distance = min_distance if (delx < 0) else -min_distance
    dx, dy = points[idx, 0], points[idx, 1] 
    return dx, dy, min_distance 

def make_pose_symbol(translation: List[float], rotation: List[float]) -> Pose:
    pose = Pose()
    pose.position.x = translation[0]
    pose.position.y = translation[1]
    pose.position.z = translation[2]
    pose.orientation.x = rotation[0]
    pose.orientation.y = rotation[1]
    pose.orientation.z = rotation[2]
    pose.orientation.w = rotation[3]
    return pose
    
def update_angular_velocity(old_velocity: Twist, d, k  = 1.7) -> Twist:
    new_velocity = old_velocity    
    new_velocity.angular.z = old_velocity.angular.z + k * d
    return new_velocity

def controller(robot: Robot, path_info: List[List[float]]):
    actions, waypoints = path_info
    rospy.sleep(3.)
    path_publisher = rospy.Publisher('path', Path, queue_size=10)
    vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    listener = tf.TransformListener()
    rospy.sleep(3.)
    path = make_path_messages(waypoints)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
            current_pose = make_pose_symbol(trans, rot)
            _, _, distance = find_closest_point(current_pose, waypoints)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("transformation unavailable between map and baselink ...")
            rospy.sleep(1.)
            continue
        action = [0,0] if (len(actions) == 0) else actions.pop(0)
        vel = make_velocity_messages(robot, action)
        #update omega
        if (abs(distance) > 0.1 and len(actions) > 0):
           vel =  update_angular_velocity(vel, distance)
        path_publisher.publish(path)
        vel_publisher.publish(vel)
        rate.sleep()