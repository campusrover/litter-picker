#!/usr/bin/env python
import rospy
import actionlib
from utils import read_waypoints
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
import math
import constants
from std_msgs.msg import Int32

map_file = rospy.get_param('~map_file')
waypoints_file = rospy.get_param('~waypoints_file')

pub = rospy.Publisher(constants.STATE_TOPIC_NAME, Int32, queue_size=1)
waypoints = read_waypoints(waypoints_file)

def get_state(msg: Int32):
    global state
    state = msg.data

# Main program starts here
if __name__ == '__main__':
    state = constants.GO_TO_NEXT_WAYPOINT
    rospy.init_node('patrol')
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    state_sub = rospy.Subscriber(constants.STATE_TOPIC_NAME, Int32, get_state)

    # wait for action server to be ready
    client.wait_for_server()

    # Loop until ^c
    while not rospy.is_shutdown():
        # repeat the waypoints over and over again
        for goal in waypoints:
            print("Going for goal: ", goal)
            client.send_goal(goal)
            client.wait_for_result()
            pub.publish(constants.REACHED_WAYPOINT)

            #wait for the rotation to be finished
            while (state != constants.GO_TO_NEXT_WAYPOINT): 
                continue
