#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
import tf
import pyrealsense2

def wrap_pi(ang):
        if ang < 0:
            while ang < 0:
                ang += 2 * np.pi
        elif ang > 2 * np.pi:
            while ang > 2 * np.pi:
                ang -= 2 * np.pi
        
        return ang

def construct_pose(x, y, z, roll , pitch, yaw, frame):
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        return pose

def convert_depth_to_phys_coord_using_realsense(x, y, depth, intrinsics): 
        result = pyrealsense2.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth) 
        #result[0]: right, result[1]: down, result[2]: forward
        return result[2], -result[0], -result[1]

def clamp(num, min_value, max_value):
   return max(min(num, max_value), min_value)