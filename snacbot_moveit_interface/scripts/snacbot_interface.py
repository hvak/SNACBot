import sys
import copy

from sympy import Quaternion

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class SNACBotMoveitInterface(object):
    def __init__(self):
        super(SNACBotMoveitInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("snacbot_moveit_interface", anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        arm_group_name = "snacbot_arm"
        self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name)

        hand_group_name = "snacbot_hand"
        self.hand_group = moveit_commander.MoveGroupCommander(hand_group_name)

        self.display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        self.planning_frame = self.arm_group.get_planning_frame()
        print("============ Planning frame: %s" % self.planning_frame)

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.arm_group.get_end_effector_link()
        print("============ End effector link: %s" % self.eef_link)

        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.group_names)

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")
    
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

    def go_to_pose_goal(self, pose_goal):

        current_pose = self.arm_group.get_current_pose()

        # constraint = moveit_msgs.msg.Constraints()
        # constraint.name = "any gripper yaw"
        # ee_constraint = moveit_msgs.msg.OrientationConstraint()
        # ee_constraint.header = current_pose.header
        # ee_constraint.link_name = "ee_link"
        # ee_constraint.orientation.x = quaternion[0]
        # ee_constraint.orientation.y = quaternion[1]
        # ee_constraint.orientation.z = quaternion[2]
        # ee_constraint.orientation.w = quaternion[3]
        # ee_constraint.absolute_x_axis_tolerance = 0.1
        # ee_constraint.absolute_y_axis_tolerance = 0.1
        # ee_constraint.absolute_z_axis_tolerance = 2 * pi
        # ee_constraint.weight = 1
        # constraint.orientation_constraints.append(ee_constraint)
        # self.arm_group.set_path_constraints(constraint)

        self.arm_group.set_pose_target(pose_goal)
        plan = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        self.arm_group.clear_path_constraints()

        current_pose = self.arm_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def execute_plan(self, plan):
        return

    def display_trajectory(self, plan):
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)

    def open_gripper(self):
        return
    
    def close_gripper(self):
        return


if __name__ == "__main__":
    snacbot_interface = SNACBotMoveitInterface()

    print("============ Generating plan 1")
    x = 0.1
    y = -0.1
    z = 0
    roll = 0
    pitch = 1.57
    yaw = 0

    pose = snacbot_interface.construct_pose(x, y, z, roll, pitch, yaw)

    print(snacbot_interface.go_to_pose_goal(pose))




