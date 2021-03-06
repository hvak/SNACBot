#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
from snacbot_common.common import clamp


from moveit_commander.conversions import pose_to_list

class SNACBotMoveitInterface(object):
    def __init__(self, debug=True):
        super(SNACBotMoveitInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)

        self.debug = debug

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        arm_group_name = "snacbot_arm"
        self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name)

        hand_group_name = "snacbot_hand"
        self.hand_group = moveit_commander.MoveGroupCommander(hand_group_name)


        self.planning_frame = self.arm_group.get_planning_frame()
        if debug: print("============ Planning frame: %s" % self.planning_frame)

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.arm_group.get_end_effector_link()
        if debug: print("============ End effector link: %s" % self.eef_link)

        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        if debug: print("============ Available Planning Groups:", self.group_names)

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        if debug: print("============ Printing robot state")
        if debug: print(self.robot.get_current_state())
        if debug: print("")

    def set_vel_accel_scaling(self, vel_scale, accel_scale):
        self.arm_group.set_max_velocity_scaling_factor(vel_scale)
        self.arm_group.set_max_acceleration_scaling_factor(accel_scale)
        
    
    #create pose msg from xyz and rpy
    def construct_pose(self, x, y, z, roll , pitch, yaw):
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose = geometry_msgs.msg.Pose()
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        return pose

    def goal_pose_valid(self, pose):

        if pose.position.x == 0 and pose.position.y == 0 and pose.position.z == 0:
            return False

        self.arm_group.set_pose_target(pose)
        plan = self.arm_group.plan()
        self.arm_group.clear_pose_targets()
        return plan[0]
    
    def goal_position_valid(self, pose):

        xyz = [pose.position.x, pose.position.y, pose.position.z]
        if xyz[0] == 0 and xyz[1] == 0 and xyz[2] == 0:
            return False

        self.arm_group.set_position_target(xyz)
        plan = self.arm_group.plan()
        self.arm_group.clear_pose_targets()
        return plan[0]

    #goes to predfined group states (as defined in srdf)
    def go_to_group_state(self, group_name):
        if self.debug: print("Going to group state: ", group_name)
        self.arm_group.set_named_target(group_name)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return success

    #go to a specified pose
    def go_to_pose_goal(self, pose_goal):
        self.arm_group.set_pose_target(pose_goal)
        plan = self.arm_group.plan()

        if plan[0]:
            if self.debug: print("Going to pose goal: ", pose_goal)
            self.arm_group.go(wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            return True
        else:
            print("Couldn't go to pose goal!")
            return False
    
    def go_to_position_goal(self, pose):
        xyz = [pose.position.x, pose.position.y, pose.position.z]
        self.arm_group.set_position_target(xyz)
        plan = self.arm_group.plan()

        if plan[0]:
            if self.debug: print("Going to position goal: ", xyz)
            self.arm_group.go(wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            return True
        else:
            print("Couldn't go to position goal!")
            return False

    def get_current_rpy(self):
        return self.arm_group.get_current_rpy()

    def rotate_waist(self, ang):
        #ang = clamp(ang, -3.14, 3.14)
        self.arm_group.set_joint_value_target("waist_joint", ang)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return success

    def get_waist_angle(self):
        return self.arm_group.get_current_joint_values()[0]

    #open gripper to a dist between 0 and 0.125m
    def open_gripper_dist(self, dist):
        rad = 14.24189 * dist + 0.37925
        rad = clamp(rad, 0.279, 2.059)
        
        joint_goal = self.hand_group.get_current_joint_values()
        joint_goal[0] = 0.33 * rad
        joint_goal[1] = -0.33 * rad
        joint_goal[2] = rad
        success = self.hand_group.go(joint_goal, wait=True)
        self.hand_group.stop()
        self.hand_group.clear_pose_targets()
        return success

    def open_gripper(self):
        if self.debug: print("Opening gripper")
        self.hand_group.set_named_target("open")
        success = self.hand_group.go(wait=True)
        self.hand_group.stop()
        self.hand_group.clear_pose_targets()
        return success
    
    def close_gripper(self):
        if self.debug: print("Closing gripper")
        self.hand_group.set_named_target("close")
        success = self.hand_group.go(wait=True)
        self.hand_group.stop()
        self.hand_group.clear_pose_targets()
        return success



