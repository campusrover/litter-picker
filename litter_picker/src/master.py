#!/usr/bin/env python3

from utils import read_waypoints, read_collection_site, read_config
import rospy


class LitterPickerState:

    def __init__(self, waypoints_site_goal, collection_site_goal):
        self.waypoints = waypoints_site_goal
        self.current_waypoint_index = 0

        self.collection_site = collection_site_goal
        self.number_of_trash_picked = 0


class LitterPicker:

    def __init__(self, task, state: LitterPickerState):
        self.current_task = task
        self.state = state

    def execute(self):
        self.current_task.start()
        self.current_task = self.current_task.next()


if __name__ == '__main__':
    from navigation import NavigationTask

    rospy.init_node("litter_picker")

    waypoints, collection_site = read_config(rospy.get_param('~waypoints_file'))
    waypoints, collection_site = read_waypoints(waypoints), read_collection_site(collection_site)

    litter_picker_state = LitterPickerState(waypoints, collection_site)
    litter_picker = LitterPicker(NavigationTask(litter_picker_state), litter_picker_state)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        litter_picker.execute()
        rate.sleep()
