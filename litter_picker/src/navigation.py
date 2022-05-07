import rospy
from move_base_msgs.msg import MoveBaseGoal
from utils import navigate_to_waypoint

from task import Task


class NavigationTask(Task):
    """
    Handle the task of moving the litter picker to different waypoints around the map
    """

    def __init__(self, state):
        """
        Constructor for the Navigation Task

        :param state: an instance of the LitterPickerState, used by the started() method to determine
        which waypoint to navigate to
        """
        super().__init__(state)

    def start(self):
        """
        Execute the navigation task. Send a goal to the move_base actionlib server to move the litter picker
        to the next waypoint on the map
        """
        waypoint: MoveBaseGoal = self.state.waypoints[self.state.current_waypoint_index]

        rospy.loginfo("[Navigation Task] sending the litter picker to location {}, {}".format(
            waypoint.target_pose.pose.position.x, waypoint.target_pose.pose.position.y))

        if navigate_to_waypoint(waypoint):
            self.has_succeeded = True
            rospy.loginfo("[Navigation Task] has successfully reached location {}, {}".format(
                waypoint.target_pose.pose.position.x, waypoint.target_pose.pose.position.y))
        else:
            rospy.logwarn("[Navigation Task] has failed reached location {}, {}: try again".format(
                waypoint.target_pose.pose.position.x, waypoint.target_pose.pose.position.y))

    def next(self):
        """
        Determine the next task to be executed. If the task has been executed successfully, then return
        an instance of the RotationTask so that robot can rotate. Otherwise, try to navigate to the given
        waypoint again.

        :return either an instance of RotationTask or a new instance of NavigationTask
        """
        from rotation import RotationTask

        if self.has_succeeded:
            self.state.current_waypoint_index = (self.state.current_waypoint_index + 1) % len(
                self.state.waypoints)
            return RotationTask(self.state)
        else:
            return NavigationTask(self.state)
