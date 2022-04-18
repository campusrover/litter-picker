#!/usr/bin/env python3
import math

import rospy
import states
from move_base_msgs.msg import MoveBaseGoal
from std_msgs.msg import Float32, Int32, String
from utils import read_waypoints

# The states of the cleaner
GO_TO_NEXT_WAYPOINT_STATE = 0
WAITING_FOR_ROBOT_TO_REACH_WAYPOINT = 1
AT_A_WAYPOINT = 2
WAITING_FOR_ROBOT_TO_FINISH_ROTATE = 3
AT_A_WAYPOINT_NEEDS_PROCESSING_IMAGE = 4
WAITING_FOR_NAVIGATION_TO_START = 5
WAITING_FOR_ROTATION_TO_START = 6

# States description
states_description = [
    "Preparing the robot to move toward the next waypoint",
    "Waiting for the robot to reach the waypoint",
    "Currently at a waypoint",
    "Waiting for the robot to finish rotating",
    "Processing image to detect trash",
    "Waiting for the navigation node to start",
    "Waiting for the rotation node to start"
]


class LitterPicker:

    def __init__(self):
        rospy.init_node("litter_picker")
        self.state = GO_TO_NEXT_WAYPOINT_STATE
        self.status = {'navigation': states.AVAILABLE, 'rotation': states.AVAILABLE}
        self.waypoints = read_waypoints(rospy.get_param('~waypoints_file'))

        # publisher for its internal state
        self.state_pub = rospy.Publisher('master/state', String, queue_size=1)

        # information about navigation
        self.nav_goal_pub = rospy.Publisher('navigation/goal', MoveBaseGoal, queue_size=1)
        self.nav_sub = rospy.Subscriber('navigation/status', Int32,
                                        self._create_status_callback("navigation"))

        self.next_waypoint_index = 0
        self.next_waypoint = self.waypoints[self.next_waypoint_index]

        # information about rotation
        self.rotation_goal_pub = rospy.Publisher('rotation/goal', Float32, queue_size=1)
        self.nav_sub = rospy.Subscriber('rotation/status', Int32,
                                        self._create_status_callback('rotation'))
        self.rotation_points = [0, math.pi / 2, math.pi, 3 * math.pi / 2]
        self.next_rotation_point = 0

    def _create_status_callback(self, node_name):

        def _callback(msg):
            self.status[node_name] = msg.data

        return _callback

    def _go_to_next_waypoint(self):
        next_waypoint = self.next_waypoint
        self.nav_goal_pub.publish(next_waypoint)
        self.state = WAITING_FOR_NAVIGATION_TO_START

    def _waiting_for_navigation_to_start(self):
        if self.status['navigation'] == states.IN_PROGRESS \
            or self.status['navigation'] == states.DONE:
            self.state = WAITING_FOR_ROBOT_TO_REACH_WAYPOINT
        else:
            self.nav_goal_pub.publish(self.next_waypoint)

    def _reached_waypoint(self):
        if self.status['navigation'] == states.DONE:
            rospy.loginfo("reached destination")
            self.state = AT_A_WAYPOINT
            if self.next_waypoint_index + 1 >= len(self.waypoints):
                self.next_waypoint_index = 0
            else:
                self.next_waypoint_index = self.next_waypoint_index + 1
            self.next_waypoint = self.waypoints[self.next_waypoint_index]
            self._reset_nodes_status()
        else:
            target_pos = self.next_waypoint.target_pose.pose.position
            rospy.loginfo("waiting for the navigation node to reach ({}, {})".format(
                target_pos.x, target_pos.y))

    def _rotate(self):
        next_rotation_point = self.next_rotation_point
        self.rotation_goal_pub.publish(self.rotation_points[next_rotation_point])
        self.state = WAITING_FOR_ROTATION_TO_START

    def _waiting_for_rotation_to_start(self):
        if self.status['rotation'] == states.IN_PROGRESS \
            or self.status['rotation'] == states.DONE:
            self.state = WAITING_FOR_ROBOT_TO_FINISH_ROTATE
        else:
            self.rotation_goal_pub.publish(self.rotation_points[self.next_rotation_point])

    def _finished_rotating(self):
        if self.status['rotation'] == states.DONE:
            rospy.loginfo("finished rotating")
            self.state = AT_A_WAYPOINT
            self.next_rotation_point = self.next_rotation_point + 1
            # we have finished rotating all the points
            if self.next_rotation_point >= len(self.rotation_points):
                self.state = GO_TO_NEXT_WAYPOINT_STATE
                self.next_rotation_point = 0
            else:
                self.state = AT_A_WAYPOINT
            self._reset_nodes_status()
        else:
            rospy.loginfo("waiting for robot to rotate to " +
                          str(self.rotation_points[self.next_rotation_point]))

    def _reset_nodes_status(self):
        self.status["rotation"] = 0
        self.status['navigation'] = 0

    def perform_action(self):
        self.state_pub.publish(states_description[self.state])
        if self.state == GO_TO_NEXT_WAYPOINT_STATE:
            self._go_to_next_waypoint()
        elif self.state == WAITING_FOR_ROBOT_TO_REACH_WAYPOINT:
            self._reached_waypoint()
        elif self.state == WAITING_FOR_NAVIGATION_TO_START:
            self._waiting_for_navigation_to_start()
        elif self.state == AT_A_WAYPOINT:
            self._rotate()
        elif self.state == WAITING_FOR_ROTATION_TO_START:
            self._waiting_for_rotation_to_start()
        elif self.state == WAITING_FOR_ROBOT_TO_FINISH_ROTATE:
            self._finished_rotating()


if __name__ == '__main__':
    litter_picker = LitterPicker()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        litter_picker.perform_action()
        rate.sleep()
