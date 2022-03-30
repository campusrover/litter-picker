#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import states
from std_msgs.msg import Int32

NAVIGATION_SUCCESS_CODE = 3


class NavigationNode:

    def __init__(self):
        rospy.init_node("patrol")

        self.state = states.AVAILABLE
        self.goal = []
        self.state_pub = rospy.Publisher('navigation/status', Int32, queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.goal_sub = rospy.Subscriber('navigation/goal', MoveBaseGoal, self._state_callback())
        self.client.wait_for_server()

    def _state_callback(self):

        def cb(msg):
            # do not accept new goal when haven't finished processing the existing goal
            if len(self.goal) == 0:
                self.goal.append(msg)

        return cb

    def perform_action(self):
        # only process when we have a goal and the state is available
        if not len(self.goal) == 0 and self.state == states.AVAILABLE:
            self.state = states.IN_PROGRESS
            self.client.send_goal(self.goal[0])
            self.state_pub.publish(states.IN_PROGRESS)
        elif self.state == states.IN_PROGRESS:
            if self.client.get_state() == NAVIGATION_SUCCESS_CODE:
                self.goal.pop(0)
                self.state = states.AVAILABLE
                self.state_pub.publish(states.DONE)
            else:
                # continue to publish to indicate it is still in progress
                self.state_pub.publish(states.IN_PROGRESS)


# Main program starts here
if __name__ == '__main__':
    nav = NavigationNode()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        nav.perform_action()
        rate.sleep()
