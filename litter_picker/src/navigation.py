from master import LitterPickerState
from task import Task, TaskStatus
from litter_picker.msg import Trash


class Navigation(Task):

    def __init__(self, state: LitterPickerState):
        super().__init__(state)

    def start(self):
        pass

    def next(self):
        pass
