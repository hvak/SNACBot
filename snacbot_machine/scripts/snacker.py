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
        return State.SEARCH_HUMAN

        

    #Look around for a human by panning one time.
    #Needs to switch directions. We should only break this state when we have found a human.
    #RETURNS State.SEARCH_FOOD ALWAYS.
    def scan_for_human_state(self):
        human_found = False
        while not human_found:
            #Look for human with mouth open. If human exists, do:
            if ???
                human_found = True
                self.last_human_position = ???
        return State.SEARCH_FOOD:

    def get_grasp_pose(self):
        return self.snacbot.construct_pose(0.2, -0.2, 0.02, 0, 1.57, 0)


    #Look for food by going to food search pose.
    #
    #IF there is no food found, do:
    #   1. Go to sleep pose
    #   2. SLEEP: Set a timer for some amount of time to wait.
    #   3. Return State.SEARCH_FOOD
    #
    #IF THERE IS FOOD, DO:
    #   1. self.food_pose = food_pose
    #   2. Return State.GRASP
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

    #TODO: PHASE THIS OUT
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

    #GO TO PREGRASP POSE, THEN GO TO GRAB FOOD. FOOD WILL BE LOCATED AT self.food_pose
    #RETURN TO self.last_human_position
    #Unconditionally return State.SEARCH_FACE
    def grasp_state(self):
        state = self.snacbot.go_to_pose_goal(self.grasp_pose)
        if not state: return StateResult.FAIL
        state = self.snacbot.close_gripper()
        if not state: return StateResult.FAIL

        return StateResult.SUCCESS        


    #IN THIS STATE, SHOULD WE PAN AROUND?
    #Wait for a human to be opening their mouth for some specified amount of time
    #May be able to do a smaller FSM inside of here to search for humans within range like before
    #Use the detection buffer to make it resistant to errors kinda
    #self.feed_pose is the last known position of the human, update it just before we return the next state
    #Unconditionally returns State.FEED
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

    #TODO: MERGE WITH feed_state()
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

    #Feed the human by going to self.feed_pose
    #Go to sleep pose
    #Sleep for some amount of time using a timer
    #return State.SCAN_HUMAN
    def feed_state(self):
        state = self.snacbot.go_to_position_goal(self.feed_pose)
        if not state: return StateResult.FAIL
        state = self.snacbot.open_gripper()
        if not state: return StateResult.FAIL

        return StateResult.SUCCESS

    def go_to_sleep_pose(self):
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
                current_state = self.initial_state()

            #SCAN FOR HUMAN STATE
            elif current_state == State.SEARCH_HUMAN:
                current_state = self.scan_for_human_state()

            #SEARCH FOOD STATE
            elif current_state == State.SEARCH_FOOD:
                current_state = self.search_food_state()
            
            #GRASP STATE
            elif current_state == State.GRASP:
                current_state = self.grasp_state()

            #SEARCH FACE STATE
            elif current_state == State.SEARCH_FACE:
                current_state = self.search_mouth_state()

            #POST GRASP STATE
            elif current_state == State.FEED:
                current_state = self.feed_state()
            

            #TODO: add more states

            else:
                self.sleep_state()
            
            current_state = next_state



if __name__ == "__main__":
    snacker = Snacker()
    snacker.run()
