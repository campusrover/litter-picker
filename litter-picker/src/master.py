#!/usr/bin/env python3
import rospy
from move_base_msgs.msg import MoveBaseGoal
from std_msgs.msg import Float32, Int32
from utils import read_waypoints
import math

# The states of the cleaner
GO_TO_NEXT_WAYPOINT_STATE = 0
WAITING_FOR_ROBOT_TO_REACH_WAYPOINT = 1
AT_A_WAYPOINT = 2
WAITING_FOR_ROBOT_TO_FINISH_ROTATE = 3
AT_A_WAYPOINT_NEEDS_PROCESSING_IMAGE = 4


class LitterPicker:
    def __init__(self, way_points):
        self.state = GO_TO_NEXT_WAYPOINT_STATE
        self.status = {'navigation': 0, 'rotation': 0}
        self.waypoints = way_points

        # information about navigation
        self.nav_goal_pub = rospy.Publisher('navigation/goal', MoveBaseGoal, queue_size=1)
        self.nav_sub = rospy.Subscriber(
            'navigation/status', Int32,
            self._create_status_callback("navigation")
        )
        
        self.next_waypoint_index = 0
        self.next_waypoint = self.waypoints[self.next_waypoint_index]

        # information about rotation
        self.rotation_goal_pub = rospy.Publisher('rotation/goal', Float32, queue_size=1)
        self.nav_sub = rospy.Subscriber(
            'rotation/status', Int32, 
            self._create_status_callback('rotation')
        )
        self.rotation_points = [0, math.pi / 2, math.pi, -math.pi / 2]
        self.next_rotation_point = 0

    def _create_status_callback(self, node_name):
        def _callback(msg):
            self.status[node_name] = msg.data

        return _callback

    def _go_to_next_waypoint(self):
        next_waypoint = self.next_waypoint
        self.nav_goal_pub.publish(next_waypoint)
        if self.status['navigation'] == 1:
            self.state = WAITING_FOR_ROBOT_TO_REACH_WAYPOINT

    def _reached_waypoint(self):
        if self.status['navigation'] == 1:
            self.state = AT_A_WAYPOINT
            if (self.next_waypoint_index + 1 >= len(self.waypoints)):
                self.next_waypoint_index = 0
            else: 
                self.next_waypoint_index = self.next_waypoint_index + 1
            self.next_waypoint = self.waypoints[self.next_waypoint_index]

    def _rotate(self):
        next_rotation_point = self.next_rotation_point
        self.rotation_goal_pub.publish(math.pi / 4)
        self.state = WAITING_FOR_ROBOT_TO_FINISH_ROTATE

    def _finished_rotating(self):
        if self.status['rotation'] == 1:
            self.state = AT_A_WAYPOINT
            self.next_rotation_point = self.next_rotation_point + 1
            if self.next_rotation_point == len(self.rotation_points):
                self.state = AT_A_WAYPOINT_NEEDS_PROCESSING_IMAGE
                self.next_rotation_point = 0

    def perform_action(self):
        if self.state == GO_TO_NEXT_WAYPOINT_STATE:
            self._go_to_next_waypoint()
        elif self.state == WAITING_FOR_ROBOT_TO_REACH_WAYPOINT:
            self._reached_waypoint()
        elif self.state == AT_A_WAYPOINT:
            self.state = GO_TO_NEXT_WAYPOINT_STATE
            #self._rotate()
        # elif self.state == WAITING_FOR_ROBOT_TO_FINISH_ROTATE:
        #     self._finished_rotating()
        elif self.state == AT_A_WAYPOINT_NEEDS_PROCESSING_IMAGE:
            self.state = GO_TO_NEXT_WAYPOINT_STATE


if __name__ == '__main__':
    rospy.init_node('litter-picker')
    waypoints_file = rospy.get_param('~waypoints_file')
    litter_picker = LitterPicker(read_waypoints(waypoints_file))

    while not rospy.is_shutdown():
        litter_picker.perform_action()
