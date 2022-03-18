#!/usr/bin/env python3
import rospy
import actionlib
from utils import read_waypoints
from move_base_msgs.msg import MoveBaseAction
import constants
from std_msgs.msg import Int32


def get_goal(msg):
    global goal
    goal = msg.data
    global received_goal
    received_goal = True


# Main program starts here
if __name__ == '__main__':
    goal = MoveBaseGoal()
    rospy.init_node('patrol')

    state_pub = rospy.Publisher('navigation/status', Int32, queue_size=1)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, get_goal)
    
    # wait for action server to be ready
    client.wait_for_server()

    received_goal = False

    # Loop until ^c
    while not rospy.is_shutdown():
        if not received_goal:
            client.send_goal(goal)
            received_goal = True
            should_navigate = True
            state_pub.publish(0)
        else: 
            if client.get_goal_status_text() == 'SUCCEEDED':
                state_pub.publish(1)
                received_goal = False 
                should_navigate = False 
            else: 
                state_pub.publish(0)


