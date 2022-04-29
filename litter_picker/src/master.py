#!/usr/bin/env python3
import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String

import actions
from litter_picker.msg import NavigationAction, RotationAction, RotationGoal, NavigationGoal, TrashAction, TrashGoal, \
    TrashResult
from darknet_ros_msgs.msg import BoundingBoxes
from utils import read_waypoints
import topics

# the state of the Litter Picker
GO_TO_WAYPOINT = 0
ROTATE_TOWARD_TRASH = 1
TRASH_LOCALIZATION = 2

# failed state of the actionlib
FAILED_STATES = {GoalStatus.RECALLED, GoalStatus.REJECTED, GoalStatus.ABORTED, GoalStatus.PREEMPTED}


class LitterPicker:

    def __init__(self):
        rospy.init_node('litter_picker')
        self.state_to_action = {
            GO_TO_WAYPOINT: self.go_to_waypoint,
            ROTATE_TOWARD_TRASH: self.rotate,
            TRASH_LOCALIZATION: self.get_location_of_trash,
        }

        # should try to move toward a waypoint at the beginning
        self.state = GO_TO_WAYPOINT

        # the state of the waypoint
        self.waypoints = read_waypoints(rospy.get_param('~waypoints_file'))
        self.next_waypoint = 0

        # state of the bonding box
        self.object_count = 0
        self.box_id = 0
        self.trash_location = (0, 0, 0)

        # actionlib clients
        self.navigation_client = actionlib.SimpleActionClient(actions.NAVIGATION_ACTION,
                                                              NavigationAction)
        self.rotation_client = actionlib.SimpleActionClient(actions.ROTATION_ACTION, RotationAction)
        self.trash_localizer_client = actionlib.SimpleActionClient(actions.TRASH_ACTION,
                                                                   TrashAction)

        self.navigation_client.wait_for_server()
        self.rotation_client.wait_for_server()
        self.trash_localizer_client.wait_for_server()

        # publisher for its current state for the GUI to use
        self.state_pub = rospy.Publisher(topics.STATE_TOPIC, String, queue_size=1)

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
            self.state = ROTATE_TOWARD_TRASH
        elif self.navigation_client.get_state() in FAILED_STATES:
            rospy.loginfo("[Master:] Fail to get to the expected waypoint, retrying")

    def rotate(self):
        self.state_pub.publish(String("Rotate to detect trash"))

        self.rotation_client.send_goal(RotationGoal())
        self.rotation_client.wait_for_result()

        if self.rotation_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("[Master:] Rotated toward trash, ready to move toward it")
            self.state = TRASH_LOCALIZATION

            rotation_goal = self.rotation_client.get_result()
            self.box_id = rotation_goal.box_id

        elif self.rotation_client.get_state() in FAILED_STATES:
            rospy.loginfo("No trash, go to next way point instead")
            self.state_pub.publish(String("No trash found, go the next waypoint instead"))
            self.state = GO_TO_WAYPOINT

    def get_location_of_trash(self):
        trash_goal = TrashGoal()
        trash_goal.box_id = self.box_id

        self.state_pub.publish("Trying to move toward the trash with box_id {}".format(self.box_id))

        self.trash_localizer_client.send_goal(TrashGoal())
        self.trash_localizer_client.wait_for_result()

        if self.trash_localizer_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("[Master: ] Got the trash, rotating to see if there are trash nearby")
            self.state = ROTATE_TOWARD_TRASH

        elif self.trash_localizer_client.get_state() in FAILED_STATES:
            rospy.loginfo("[Master: ] Lost track of the trash, rotating to rediscover the trash")
            self.state = ROTATE_TOWARD_TRASH

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
