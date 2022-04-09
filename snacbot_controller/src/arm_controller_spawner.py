#!/usr/bin/env python
import sys
import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)

from trajectory_msgs.msg import  JointTrajectory, JointTrajectoryPoint

class JointTrajectoryActionServer(object):

    def __init__(self, controller_name):
        self.dx_trajectory_pub = rospy.Publisher ("/joint_trajectory", JointTrajectory, queue_size=10)  
        self._action_ns = controller_name + '/follow_joint_trajectory'
        rospy.loginfo('Starting controller: {}'.format(self._action_ns))
        self._as = actionlib.SimpleActionServer(
                self._action_ns,
                FollowJointTrajectoryAction,
                execute_cb=self.execute_cb,
                auto_start = False)
        self._action_name = rospy.get_name()
        self._as.start()
        rospy.loginfo('Controller {} has been successfuly started.'.format(controller_name))
        self._feedback = FollowJointTrajectoryFeedback
        self._result = FollowJointTrajectoryResult

        
    def execute_cb(self, goal):
        rospy.loginfo('Receving controller trajectories...')
        jt = JointTrajectory()
        jt.header.stamp = rospy.Time.now()
        jt.joint_names = list(goal.trajectory.joint_names)
        
        #make sure shoulder joint exists
        shoulder_idx = -1
        i = 0
        for joint in jt.joint_names:
            if joint == "shoulder_joint":
                shoulder_idx = i
                break
            i += 1

        #add shadow joint if so
        if shoulder_idx != -1:
            jt.joint_names.append("shoulder_shadow_joint")

        jt.points = []
        for point in goal.trajectory.points:

            jtp = JointTrajectoryPoint()
            jtp.positions = list(point.positions)
            jtp.velocities = list(point.velocities)
            jtp.accelerations = list(point.accelerations)
            jtp.effort = list(point.effort)
            jtp.time_from_start = point.time_from_start

            #add negative of shoulder joint so shadow joint moves opposite
            if shoulder_idx != -1:
                jtp.positions.append(-jtp.positions[shoulder_idx])
                jtp.velocities.append(-jtp.velocities[shoulder_idx])
                jtp.accelerations.append(-jtp.accelerations[shoulder_idx])

            jt.points.append(jtp)

        self.dx_trajectory_pub.publish(jt)
        result_ = FollowJointTrajectoryResult()
        self._as.set_succeeded(result_)
        rospy.loginfo('Joint trajectories has been successfuly transmitted to dynamixel branch.')


if __name__ == '__main__':
    rospy.init_node('snacbot_controller_interface')
    rospy.loginfo("Starting SNACBot trajectory controller interface.")
    if len(sys.argv) < 2:
        rospy.loginfo("Please! Set a controller_name using args.")
        exit()
    controller_name = sys.argv[1]
    server = JointTrajectoryActionServer(controller_name)
    # servers = []
    # for controller_name in sys.argv[2:]:
        
    #     servers.append(server)
    rospy.spin()
