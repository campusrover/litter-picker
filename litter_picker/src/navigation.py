#!/usr/bin/env python3
import rospy
import actionlib
from actionlib import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

import actions
from litter_picker.msg import NavigationAction, NavigationGoal, NavigationResult


class NavigationActionServer:

    def __init__(self, name):
        self.server = actionlib.SimpleActionServer(name,
                                                   NavigationAction,
                                                   execute_cb=self.create_goal_cb,
                                                   auto_start=False)
        self.navigation_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.navigation_client.wait_for_server()
        self.server.start()
        self.state_pub = rospy.Publisher('master/state', String, queue_size=1)
        self.result = NavigationResult()

    def create_goal_cb(self, goal: NavigationGoal):
        goal_pose = MoveBaseGoal()

        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose.position.x = goal.x
        goal_pose.target_pose.pose.position.y = goal.y
        goal_pose.target_pose.pose.position.z = goal.z

        # doesn't matter since, it's going to rotate for 360 degrees anyway
        goal_pose.target_pose.pose.orientation.x = 0
        goal_pose.target_pose.pose.orientation.y = 0
        goal_pose.target_pose.pose.orientation.z = 0
        goal_pose.target_pose.pose.orientation.w = 1

        self.navigation_client.send_goal(goal_pose)
        self.navigation_client.wait_for_result()

        if self.navigation_client.get_state() == GoalStatus.SUCCEEDED:
            self.result.msg = "[Navigation Node]: Reached location {}, {}".format(goal.x, goal.y)
            self.server.set_succeeded(self.result)
        else:
            self.result.msg = "[Navigation Node]: Failed to reach location {}, {}".format(
                goal.x, goal.y)
            self.server.set_aborted(self.result)

    def cancel_goal(self):
        self.navigation_client.cancel_all_goals()


# Main program starts here
if __name__ == '__main__':
    rospy.init_node('navigation')
    rospy.loginfo("Navigation Node is running")
    nav = NavigationActionServer(actions.NAVIGATION_ACTION)

    # make sure that the goal is cancelled when shut down occurs
    rospy.on_shutdown(nav.cancel_goal)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
