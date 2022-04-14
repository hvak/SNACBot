from snacbot_interface import SNACBotMoveitInterface
from geometry_msgs.msg import Pose
from enum import Enum

class StateResult(Enum):
    SUCCESS = 0
    IN_PROGRESS = 0
    PLANNING_FAIL = 1
    GRIPPER_FAIL = 2
    OTHER_FAIL = 3

class State(Enum):
    INITIAL_STATE = 0
    SEARCH_FOOD = 1
    PRE_GRASP = 2
    GRASP = 3
    POST_GRASP = 4

    FEED_STATE = 5
    SLEEP_STATE = 6

    COMPLETE = 7



class Snacker:
    def __init__(self):
        self.snacbot = SNACBotMoveitInterface(debug=False)
        self.grasp_pose = Pose()
        self.pre_grasp_pose = Pose()

    def initial_state(self):
        self.snacbot.go_to_group_state("sleep")
        return StateResult.SUCCESS

    def get_grasp_pose(self):
        return self.snacbot.construct_pose(0.2, -0.2, 0.02, 0, 1.57, 0)


    def search_food_state(self):
        #go to food state
        state = self.snacbot.go_to_group_state("food")
        if not state: return StateResult.PLANNING_FAIL

        #calculate grasp pose
        self.grasp_pose = self.get_grasp_pose()

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

        state = self.snacbot.go_to_pose_goal(self.pre_grasp_pose)
        if not state: return StateResult.PLANNING_FAIL
        
        #open gripper
        state = self.snacbot.open_gripper()
        if not state: return StateResult.GRIPPER_FAIL

        return StateResult.SUCCESS

    def grasp_state(self):
        state = self.snacbot.go_to_pose_goal(self.grasp_pose)
        if not state: return StateResult.PLANNING_FAIL
        state = self.snacbot.close_gripper()
        if not state: return StateResult.GRIPPER_FAIL

        return StateResult.SUCCESS


    def post_grasp_state(self):
        state = self.snacbot.go_to_pose_goal(self.pre_grasp_pose)
        if not state: return StateResult.PLANNING_FAIL

        return StateResult.SUCCESS


    #TODO: other states
    def feed_state(self):
        self.snacbot.go_to_group_state("home")

    def sleep_state(self):
        self.snacbot.close_gripper()
        self.snacbot.go_to_group_state("sleep")

    #run entire state machine
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
                    next_state = State.SLEEP_STATE
                elif ret == StateResult.IN_PROGRESS:
                    next_state = State.POST_GRASP
            

            #TODO: add more states

            else:
                self.sleep_state()
            
            current_state = next_state



if __name__ == "__main__":
    snacker = Snacker()

    snacker.run()
