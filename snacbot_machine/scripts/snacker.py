#!/usr/bin/env python3


from geometry_msgs.msg import Pose
from enum import Enum
from snacknet.srv import *
from face_detection.srv import *
import rospy
import tf
from snacbot_common.snacbot_moveit_interface import *
from snacbot_common.common import clamp
import numpy as np
import time
from math import atan

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
        self.last_human_angle = 0
        self.snacbot.set_vel_accel_scaling(1.0, 1.0)

    def initial_state(self):
        self.snacbot.go_to_group_state("sleep")
        self.snacbot.open_gripper()
        self.snacbot.close_gripper()
        return State.SEARCH_HUMAN

        

    #Look around for a human by panning one time.
    #Needs to switch directions. We should only break this state when we have found a human.
    #RETURNS State.SEARCH_FOOD ALWAYS.
    def scan_for_human_state(self):
        self.snacbot.go_to_group_state("sleep")
        human_found = False
        cur_ang = 0
        dir = -1
        while not human_found:

            cur_ang += dir * 15.0 * np.pi / 180.0
            if cur_ang > np.pi / 2:
                dir *= -1
            elif cur_ang < -np.pi / 2:
                dir *= -1
            print(cur_ang)

            #Look for human with mouth open. If human exists, do:

            face_pose = Pose()
            rospy.wait_for_service('Face_Service')
            try:
                face_detect = rospy.ServiceProxy('Face_Service', FaceService)
                face_pose = face_detect().data.pose
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))

            #human is found if face pose reachable
            if self.snacbot.goal_position_valid(face_pose):
                human_found = True
                if face_pose.position.x > 0:
                    self.last_human_angle = atan(face_pose.position.y / face_pose.position.x)
                else:
                    self.last_human_angle = self.snacbot.get_waist_angle()
                print("Found human at angle:", self.last_human_angle)
            else:
                self.snacbot.rotate_waist(cur_ang)

            

            

        return State.SEARCH_FOOD


    #Look for food by going to food search pose.
    #
    #IF there is no food found, do:
    #   1. Go to sleep pose
    #   2. SLEEP: Set a timer for some amount of time to wait.
    #   3. Return State.SEARCH_HUMAN
    #
    #IF THERE IS FOOD, DO:
    #   1. self.food_pose = food_pose
    #   2. Return State.GRASP
    def search_food_state(self):
        #go to food state
        state = self.snacbot.go_to_group_state("food")

        food_pose = Pose()
        rospy.wait_for_service('Snack_Grab')
        try:
            snack_grab = rospy.ServiceProxy('Snack_Grab', SnackGrabService)
            food_pose = snack_grab().data.pose
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        #no food, search for human again
        if not self.snacbot.goal_pose_valid(food_pose):
            #self.snacbot.go_to_group_state("sleep")
            #rospy.sleep(30.0)
            #time.sleep(30)
            return State.SEARCH_HUMAN
        
        self.grasp_pose = food_pose
        
        self.pre_grasp_pose = Pose()
        self.pre_grasp_pose.position.x = self.grasp_pose.position.x
        self.pre_grasp_pose.position.y = self.grasp_pose.position.y
        self.pre_grasp_pose.position.z = self.grasp_pose.position.z + 0.1
        self.pre_grasp_pose.orientation.x = self.grasp_pose.orientation.x
        self.pre_grasp_pose.orientation.y = self.grasp_pose.orientation.y
        self.pre_grasp_pose.orientation.z = self.grasp_pose.orientation.z
        self.pre_grasp_pose.orientation.w = self.grasp_pose.orientation.w

        #go to pregrasp
        self.snacbot.go_to_pose_goal(self.pre_grasp_pose)
        self.snacbot.open_gripper_dist(0.1)

        return State.GRASP

    #GO TO PREGRASP POSE, THEN GO TO GRAB FOOD. FOOD WILL BE LOCATED AT self.grasp_pose
    #RETURN TO self.last_human_position
    #Unconditionally return State.SEARCH_FACE
    def grasp_state(self):
        self.snacbot.go_to_pose_goal(self.grasp_pose)
        self.snacbot.close_gripper()   
        return State.SEARCH_FACE


    #IN THIS STATE, SHOULD WE PAN AROUND?
    #Wait for a human to be opening their mouth for some specified amount of time
    #May be able to do a smaller FSM inside of here to search for humans within range like before
    #Use the detection buffer to make it resistant to errors kinda
    #self.feed_pose is the last known position of the human, update it just before we return the next state
    #Unconditionally returns State.FEED
    def search_mouth_state(self):
        self.snacbot.go_to_group_state("sleep")
        self.snacbot.rotate_waist(self.last_human_angle)

        for i in range(25):
            print("Try ", i, " to find person to feed")
            mouth_pose = Pose()
            rospy.wait_for_service('Face_Service')
            try:
                face_detect = rospy.ServiceProxy('Face_Service', FaceService)
                mouth_pose = face_detect().data.pose
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
    
            found_mouth = self.snacbot.goal_position_valid(mouth_pose)
            if  found_mouth:
                self.feed_pose = mouth_pose
                return State.FEED
    
            
        print("Couldn't find person, returning food")
        self.snacbot.go_to_pose_goal(self.pre_grasp_pose)
        self.snacbot.open_gripper()
        self.snacbot.close_gripper()

        return State.SEARCH_HUMAN
        #call service

    #Feed the human by going to self.feed_pose
    #Go to sleep pose
    #Sleep for some amount of time using a timer
    #return State.SEARCH_HUMAN
    def feed_state(self):
        self.snacbot.go_to_position_goal(self.feed_pose)
        self.snacbot.open_gripper_dist(0.06)
        self.snacbot.go_to_group_state("sleep")
        self.snacbot.close_gripper()
        return State.SEARCH_HUMAN

    def go_to_sleep_pose(self):
        self.snacbot.close_gripper()
        self.snacbot.go_to_group_state("sleep")
        print("SLEEPING")
        time.sleep(30.0)
        return State.SEARCH_HUMAN

    #run entire state machinefeed_pose
    def run(self):

        current_state = State.INITIAL_STATE
        prev_state = State.INITIAL_STATE

        while (current_state != State.COMPLETE):
            
            print("Current State: ", current_state)

            #INITIAL STATE
            if current_state == State.INITIAL_STATE:
                next_state = self.initial_state()

            #SCAN FOR HUMAN STATE
            elif current_state == State.SEARCH_HUMAN:
                next_state = self.scan_for_human_state()

            #SEARCH FOOD STATE
            elif current_state == State.SEARCH_FOOD:
                next_state = self.search_food_state()
            
            #GRASP STATE
            elif current_state == State.GRASP:
                next_state = self.grasp_state()

            #SEARCH FACE STATE
            elif current_state == State.SEARCH_FACE:
                next_state = self.search_mouth_state()

            #POST GRASP STATE
            elif current_state == State.FEED:
                next_state = self.feed_state()
            

            #TODO: add more states

            else:
                self.initial_state()
            
            prev_state = current_state
            current_state = next_state



if __name__ == "__main__":
    snacker = Snacker()
    snacker.run()
