#!/usr/bin/env python3
import rospy
import actionlib
from actionlib import GoalStatus
from move_base_msgs.msg import MoveBaseAction
import actions
from litter_picker.msg import NavigationAction


class NavigationActionServer:
    def __init__(self, name):
        self.server = actionlib.SimpleActionServer(
            name,
            NavigationAction,
            execute_cb=self.create_goal_cb,
            auto_start=False
        )
        self.navigation_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.server.start()

    def create_goal_cb(self, goal):
        self.navigation_client.send_goal(goal)
        self.navigation_client.wait_for_result()

        if self.navigation_client.get_state() == GoalStatus.SUCCEEDED:
            self.server.set_succeeded(self.navigation_client.get_result())
        else:
            self.server.set_aborted(self.navigation_client.get_result())


# Main program starts here
if __name__ == '__main__':
    rospy.init_node('navigation')
    nav = NavigationActionServer(actions.NAVIGATION_ACTION)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
