#!/usr/bin/env python3
import rospy
import actionlib
from actionlib import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actions
from litter_picker.msg import NavigationAction, NavigationGoal


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

    def create_goal_cb(self, goal: NavigationGoal):
        goal_pose = MoveBaseGoal()
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
            self.server.set_succeeded(
                str.format("[Navigation Node]: Reached location {}, {}", goal.x, goal.y)
            )
        else:
            self.server.set_aborted(
                str.format("[Navigation Node]: Failed to reach location {}, {}", goal.x, goal.y)
            )


# Main program starts here
if __name__ == '__main__':
    rospy.init_node('navigation')
    rospy.loginfo("Navigation Node is running")
    nav = NavigationActionServer(actions.NAVIGATION_ACTION)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rate.sleep()
