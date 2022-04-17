#!/usr/bin/env python3

from unittest import expectedFailure
from geometry_msgs.msg import Pose
from enum import Enum
from snacknet.srv import *
from face_detection.srv import *
import rospy
import tf
from snacbot_common.snacbot_moveit_interface import *
from snacbot_common.common import clamp

class StateResult(Enum):
    SUCCESS = 0
    IN_PROGRESS = 1
    FAIL = 2

class State(Enum):
    INITIAL_STATE = 0
    SEARCH_FOOD = 1
    PRE_GRASP = 2
    GRASP = 3
    POST_GRASP = 4

    SEARCH_FACE = 5
    PRE_FEED = 6
    FEED = 7

    COMPLETE = 8
    SLEEP_STATE = 9

    SEARCH_HUMAN = 10

class detection_buffer:
    def __init__(self, num_states = 4):
        self.state = 0
        self.max_state = num_states - 1
        self.thresh = num_states/2 - 1

    def reset(self):
        self.state = 0
        
    def tick(self, detection):
        self.state = min(self.max_state, self.state + 1) if detection else max(0,self.state - 1)
        return self.state > self.thresh


class Snacker:
    def __init__(self):
        self.snacbot = SNACBotMoveitInterface(debug=False)
        self.grasp_pose = Pose()
        self.pre_grasp_pose = Pose()
        self.feed_pose = Pose()
        self.pre_feed_pose = Pose()
        self.snacbot.set_vel_accel_scaling(0.8, 1.0)

    def initial_state(self):
        self.snacbot.go_to_group_state("sleep")
        self.snacbot.open_gripper()
        self.snacbot.close_gripper()
        return StateResult.SUCCESS

    def get_grasp_pose(self):
        return self.snacbot.construct_pose(0.2, -0.2, 0.02, 0, 1.57, 0)


    def search_food_state(self):
        #go to food state
        state = self.snacbot.go_to_group_state("food")
        if not state: return StateResult.FAIL

        rospy.wait_for_service('Snack_Grab')
        try:
            snack_grab = rospy.ServiceProxy('Snack_Grab', SnackGrabService)
            pose = snack_grab().data.pose
            self.grasp_pose = pose
            #self.grasp_pose = self.snacbot.construct_pose(pose.position.x, pose.position.y, pose.position.z, 0, 1.57, 0)

            print(self.grasp_pose)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            return StateResult.FAIL

        #self.grasp_pose = self.snacbot.construct_pose(0.25, -0.06, 0.01, 0, 1.57, 0)

        #self.grasp_pose = self.get_grasp_pose()

        #TODO stay in this state if no food found yet
        if False:
            return StateResult.IN_PROGRESS

        return StateResult.SUCCESS

    def pre_grasp_state(self):
        #calc pre-grasp
        self.pre_grasp_pose = Pose()
        self.pre_grasp_pose.position.x = self.grasp_pose.position.x
        self.pre_grasp_pose.position.y = self.grasp_pose.position.y
        self.pre_grasp_pose.position.z = self.grasp_pose.position.z + 0.1
        self.pre_grasp_pose.orientation.x = self.grasp_pose.orientation.x
        self.pre_grasp_pose.orientation.y = self.grasp_pose.orientation.y
        self.pre_grasp_pose.orientation.z = self.grasp_pose.orientation.z
        self.pre_grasp_pose.orientation.w = self.grasp_pose.orientation.w

        print(self.pre_grasp_pose)

        state = self.snacbot.go_to_pose_goal(self.pre_grasp_pose)
        if not state: return StateResult.FAIL
        
        #open gripper
        state = self.snacbot.open_gripper_dist(0.06)
        if not state: return StateResult.FAIL

        return StateResult.SUCCESS

    def grasp_state(self):
        state = self.snacbot.go_to_pose_goal(self.grasp_pose)
        if not state: return StateResult.FAIL
        state = self.snacbot.close_gripper()
        if not state: return StateResult.FAIL

        return StateResult.SUCCESS


    def post_grasp_state(self):
        state = self.snacbot.go_to_pose_goal(self.pre_grasp_pose)
        if not state: return StateResult.FAIL

        return StateResult.SUCCESS

    #Look around for a human by panning
    def scan_for_human_state(self):
        


    #TODO: other states
    def search_mouth_state(self):
        ret = self.snacbot.go_to_group_state("sleep")
        if not ret: return StateResult.FAIL

        rospy.wait_for_service('Face_Service')
        try:
            face_detect = rospy.ServiceProxy('Face_Service', FaceService)
            pose = face_detect().data.pose
            self.feed_pose = pose
            print(self.feed_pose)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            return StateResult.FAIL

        return StateResult.SUCCESS
        #call service

    def pre_feed_state(self):
        
        self.pre_feed_pose = Pose()
        self.pre_feed_pose.position.x = self.feed_pose.position.x - 0.1
        self.pre_feed_pose.position.y = self.feed_pose.position.y
        self.pre_feed_pose.position.z = self.feed_pose.position.z
        self.pre_feed_pose.orientation.x = self.feed_pose.orientation.x
        self.pre_feed_pose.orientation.y = self.feed_pose.orientation.y
        self.pre_feed_pose.orientation.z = self.feed_pose.orientation.z
        self.pre_feed_pose.orientation.w = self.feed_pose.orientation.w

        print(self.pre_feed_pose)

        state = self.snacbot.go_to_position_goal(self.pre_feed_pose)
        if not state: return StateResult.FAIL
        
        #open gripper
        return StateResult.SUCCESS


    def feed_state(self):
        state = self.snacbot.go_to_position_goal(self.feed_pose)
        if not state: return StateResult.FAIL
        state = self.snacbot.open_gripper()
        if not state: return StateResult.FAIL

        return StateResult.SUCCESS

    def sleep_state(self):
        self.snacbot.close_gripper()
        self.snacbot.go_to_group_state("sleep")

    #run entire state machinefeed_pose
    def run(self):

        current_state = State.INITIAL_STATE
        next_state = State.SLEEP_STATE

        while (current_state != State.COMPLETE):
            
            print("Current State: ", current_state)

            #INITIAL STATE
            if current_state == State.INITIAL_STATE:
                ret = self.initial_state()
                #next states
                if ret == StateResult.SUCCESS:
                    next_state = State.SEARCH_FOOD
                elif ret == StateResult.IN_PROGRESS:
                    next_state = State.INITIAL_STATE

            #SEARCH FOOD STATE
            if current_state == State.SEARCH_FOOD:
                ret = self.search_food_state()
                #next states
                if ret == StateResult.SUCCESS:
                    next_state = State.PRE_GRASP
                elif ret == StateResult.IN_PROGRESS:
                    next_state = State.SEARCH_FOOD

            #PRE GRASP STATE
            elif current_state == State.PRE_GRASP:
                ret = self.pre_grasp_state()
                #next states
                if ret == StateResult.SUCCESS:
                    next_state = State.GRASP
                elif ret == StateResult.IN_PROGRESS:
                    next_state = State.PRE_GRASP
                else:
                    next_state = State.SLEEP_STATE
            
            #GRASP STATE
            elif current_state == State.GRASP:
                ret = self.grasp_state()
                #next states
                if ret == StateResult.SUCCESS:
                    next_state = State.POST_GRASP
                elif ret == StateResult.IN_PROGRESS:
                    next_state = State.GRASP
            
            #POST GRASP STATE
            elif current_state == State.POST_GRASP:
                ret = self.post_grasp_state()
                #next states
                if ret == StateResult.SUCCESS:
                    next_state = State.SEARCH_FACE
                elif ret == StateResult.IN_PROGRESS:
                    next_state = State.POST_GRASP

            #SEARCH FACE STATE
            elif current_state == State.SEARCH_FACE:
                ret = self.search_mouth_state()
                #next states
                if ret == StateResult.SUCCESS:
                    next_state = State.FEED
                elif ret == StateResult.IN_PROGRESS:
                    next_state = State.SEARCH_FACE
            
            #POST GRASP STATE
            elif current_state == State.PRE_FEED:
                ret = self.pre_feed_state()
                #next states
                if ret == StateResult.SUCCESS:
                    next_state = State.FEED
                elif ret == StateResult.IN_PROGRESS:
                    next_state = State.PRE_FEED

            #POST GRASP STATE
            elif current_state == State.FEED:
                ret = self.feed_state()
                #next states
                if ret == StateResult.SUCCESS:
                    next_state = State.INITIAL_STATE
                elif ret == StateResult.IN_PROGRESS:
                    next_state = State.FEED
                else:
                    next_state = State.SEARCH_FACE
            

            #TODO: add more states

            else:
                self.sleep_state()
            
            current_state = next_state



if __name__ == "__main__":
    snacker = Snacker()
    snacker.run()
