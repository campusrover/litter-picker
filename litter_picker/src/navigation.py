import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from utils import navigate_to_waypoint

from task import Task


class NavigationTask(Task):

    def __init__(self, state):
        super().__init__(state)
        self.move_base_client = actionlib.SimpleActionClient(
            "move_base",
            MoveBaseAction,
        )
        self.has_succeed = False
        self.move_base_client.wait_for_server()

    def start(self):
        waypoint: MoveBaseGoal = self.state.waypoints[self.state.current_waypoint_index]

        rospy.loginfo("[Navigation Task] sending the litter picker to location {}, {}".format(
            waypoint.target_pose.pose.position.x, waypoint.target_pose.pose.position.y))

        if navigate_to_waypoint(waypoint):
            rospy.loginfo("[Navigation Task] has successfully reached location {}, {}".format(
                waypoint.target_pose.pose.position.x, waypoint.target_pose.pose.position.y))
        else:
            rospy.logwarn("[Navigation Task] has failed reached location {}, {}: try again".format(
                waypoint.target_pose.pose.position.x, waypoint.target_pose.pose.position.y))

    def next(self):
        from rotation import RotationTask

        if self.has_succeed:
            self.state.current_waypoint_index = (self.state.current_waypoint_index + 1) % len(
                self.state.waypoints)
            # At this point we should create an instance of the RotationTask
            return RotationTask(self.state)
        else:
            return NavigationTask(self.state)
