#!/usr/bin/env python3
import rospy
import actionlib
from utils import read_waypoints
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import constants
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped


def get_goal(msg):
    global goal
    goal = msg
    global received_goal
    received_goal = True


# Main program starts here
if __name__ == '__main__':
    goal = MoveBaseGoal()
    rospy.init_node('patrol')

    state_pub = rospy.Publisher('navigation/status', Int32, queue_size=1)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    goal_sub = rospy.Subscriber('navigation/goal', MoveBaseGoal, get_goal)
    
    # wait for action server to be ready
    client.wait_for_server()

    received_goal = False
    state = -1

    # Loop until ^c
    while not rospy.is_shutdown() and state != 1:
        if not received_goal:
            client.send_goal(goal)
            received_goal = True
            should_navigate = True
            state = 2
        else: 
            if client.get_state() == 3:
                print("did we succeed")
                state = 1
                received_goal = False 
                should_navigate = False 
    
        state_pub.publish(state)


