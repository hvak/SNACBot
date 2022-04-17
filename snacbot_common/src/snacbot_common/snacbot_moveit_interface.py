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
        rospy.init_node("snacbot_moveit_interface", anonymous=True)

        self.debug = debug

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        arm_group_name = "snacbot_arm"
        self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name)

        hand_group_name = "snacbot_hand"
        self.hand_group = moveit_commander.MoveGroupCommander(hand_group_name)

        self.display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)

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
        self.arm_group.set_pose_target(pose)
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


    #not super useful rn, it seems pose targets display in rviz automatically
    def display_trajectory(self, plan):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan[1])
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)

    def rotate_waist(self, ang):
        ang = clamp(ang, -3.14, 3.14)
        joints = self.arm_group.get_current_joint_values()
        joints[0] = ang
        success = self.arm_group.go(joints, wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return success

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



