import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

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
        self.move_base_client.send_goal(waypoint)
        self.move_base_client.wait_for_result()

        if self.move_base_client.get_state() == GoalStatus.SUCCEEDED:
            self.has_succeed = True
            rospy.loginfo("[Navigation Task] has successfully reached location {}, {}".format(
                waypoint.target_pose.pose.position.x, waypoint.target_pose.pose.position.y))
        else:
            self.has_succeed = False
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
