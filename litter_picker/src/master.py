#!/usr/bin/env python3

from utils import read_waypoints, read_collection_site, read_config
from task import Task
import rospy


class LitterPickerState:
    """
    This class represents the current state of the litter picker, which will be passed around/shared
    by different Task instances. The Task instance may behave different given the values in the
    fields of the LitterPickerState
    """

    def __init__(self, waypoints_site_goal, collection_site_goal):
        """
        Constructor of the litter picker state

        :param waypoints_site_goal: the set of waypoints in the form of a list of MoveBaseGoal

        :param collection_site_goal: the MoveBaseGoal of the collection site
        """

        # the locations of the set of waypoints that the litter picker will visit
        self.waypoints = waypoints_site_goal

        # the next waypoint that the litter picker will visit
        self.current_waypoint_index = 0

        # the location of the collection site
        self.collection_site = collection_site_goal

        # the number of trash that the litter picker has picked up
        self.number_of_trash_picked = 0


class LitterPicker:
    """
    This class contains the main logic of the master node. It executes the task needed to be run at
    the current state
    """

    def __init__(self, task: Task):
        """
        Constructor of the LitterPicker class.

        :param task: the first task that the LitterPicker will execute
        """
        self.current_task = task

    def execute(self):
        """
        Execute the task that is needed to be run now. After the task has been finished executed, fetch
        the next task that is needed to be executed.
        """
        self.current_task.start()
        self.current_task = self.current_task.next()


if __name__ == '__main__':
    from navigation import NavigationTask

    rospy.init_node("litter_picker")

    # read the coordinates of the set of waypoints and collection set from the json config file
    waypoints, collection_site = read_config(rospy.get_param('~waypoints_file'))
    waypoints, collection_site = read_waypoints(waypoints), read_collection_site(collection_site)

    # instantiate and instance of the LitterPickerState
    litter_picker_state = LitterPickerState(waypoints, collection_site)
    litter_picker = LitterPicker(NavigationTask(litter_picker_state))
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        litter_picker.execute()
        rate.sleep()
