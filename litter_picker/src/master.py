#!/usr/bin/env python3
import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String

import actions
from litter_picker.msg import NavigationAction, RotationAction
from darknet_ros_msgs.msg import BoundingBoxes
from utils import read_waypoints

# the state of the Litter Picker
GO_TO_WAYPOINT = 0
DETECT_TRASH = 1
ROTATE_TOWARD_TRASH = 2
TRASH_LOCALIZATION = 3
MOVE_TOWARD_TRASH = 4

# failed state of the actionlib
FAILED_STATES = {
    GoalStatus.RECALLED,
    GoalStatus.REJECTED,
    GoalStatus.ABORTED,
    GoalStatus.PREEMPTED
}


class LitterPicker:

    def __init__(self):
        rospy.init_node('litter_picker')
        self.state_to_action = {
            GO_TO_WAYPOINT: self.go_to_waypoint,
            DETECT_TRASH: self.detect_trash,
            ROTATE_TOWARD_TRASH: self.rotate,
        }

        # should try to move toward a waypoint at the beginning
        self.state = GO_TO_WAYPOINT

        # the state of the waypoint
        self.waypoints = read_waypoints(rospy.get_param('~waypoints_file'))
        self.next_waypoint = 0

        # state of the bonding box
        self.object_count = 0
        self.box_sub = rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.get_object_count_cb())

        # actionlib clients
        self.navigation_client = actionlib.SimpleActionClient(actions.NAVIGATION_ACTION, NavigationAction)
        self.rotation_client = actionlib.SimpleActionClient(actions.ROTATION_ACTION, RotationAction)
        print("waiting for server")
        self.navigation_client.wait_for_server()

        # publisher for its current state
        self.state_pub = rospy.Publisher('master/state', String, queue_size=1)

        #self.rotation_client.wait_for_server()
        print("finished")

    def get_object_count_cb(self):

        def cb(msg: BoundingBoxes):
            self.object_count = msg.bounding_boxes.count()

        return cb

    def go_to_waypoint(self):
        next_waypoint = self.waypoints[self.next_waypoint]

        self.navigation_client.send_goal(next_waypoint)
        self.state_pub.publish("Moving toward waypoint ({}, {}, {})".format(
            str(next_waypoint.x), str(next_waypoint.y), str(next_waypoint.z)))
        self.navigation_client.wait_for_result()

        if self.navigation_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("[Master:] Got to the expected waypoint, detecting trash now")
            self.next_waypoint = (self.next_waypoint + 1) % (len(self.waypoints))
            self.state = GO_TO_WAYPOINT
        elif self.navigation_client.get_state() in FAILED_STATES:
            rospy.loginfo("[Master:] Fail to get to the expected waypoint, retrying")

    def detect_trash(self):
        if self.object_count > 0:
            rospy.loginfo("Trash found, rotate toward it")
            self.state = ROTATE_TOWARD_TRASH
        else:
            rospy.loginfo("Trash not found, go to the next waypoint")
            self.state = GO_TO_WAYPOINT

    def rotate(self):
        if self.navigation_client.get_state() == GoalStatus.PENDING:
            self.rotation_client.send_goal(None)

        if self.rotation_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("[Master:] Rotated toward trash, ready to move toward it")

            # need to change to TRASH_LOCALIZATION when it is ready
            self.state = GO_TO_WAYPOINT
        elif self.rotation_client.get_state() in FAILED_STATES:
            rospy.loginfo("Something went wrong, retrying")
            self.state = DETECT_TRASH

    def perform_action(self):
        if self.state not in self.state_to_action:
            rospy.logerr("Unknown state! Go back to the initial state")
            self.state = GO_TO_WAYPOINT
        else:
            (self.state_to_action.get(self.state))()


if __name__ == '__main__':
    litter_picker = LitterPicker()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        litter_picker.perform_action()
        rate.sleep()
