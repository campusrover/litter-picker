from utils import read_waypoints
from task import Task
import rospy


class LitterPickerState:
    def __init__(self, waypoints):
        waypoints = []
        current_waypoint_index = 0


class LitterPicker:
    def __init__(self, task: Task, state: LitterPickerState):
        self.current_task = task
        self.state = state

    def next(self):
        self.current_task.start()
        self.current_task = self.current_task.next()


if __name__ == '__main__':
    rospy.init_node("litter_picker")

    litter_picker_state = LitterPickerState(read_waypoints(rospy.get_param('~waypoints_file')))
    litter_picker = LitterPicker(Task(litter_picker_state), litter_picker_state)
    litter_picker.next()
