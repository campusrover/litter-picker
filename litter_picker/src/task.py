from enum import Enum
from master import LitterPickerState


class TaskStatus(Enum):
    # the task is not running
    IDLE = 1

    # the task is currently running
    IN_PROGRESS = 2

    # finished, could have failed or succeeded, handled by next() method in the Task instance
    DONE = 3


class Task:
    def __init__(self, state: LitterPickerState):
        """
        constructor fo the task

        :param state: this represents the state of the previous task execution, as the task
        instance doesn't remember anything, it provides information for the task needed to perform
        correctly at the current moment.
        """
        self.state = state

    def start(self):
        """
        when this method is called, the task will be started. If it already started, do nothing
        """
        raise NotImplementedError("not implemented")

    def next(self):
        """
        return the next task that it should be run.

        :return: a Task class object that can be instantiated
        """
        raise NotImplementedError("not implemented")

    def status(self) -> TaskStatus:
        """
        return the current status of the class.

        :return: an instance of TaskStatus
        """
