from master import LitterPickerState


class Task:

    def __init__(self, state: LitterPickerState):
        """
        constructor fo the task

        :param state: this represents the state of the previous task execution, as the task
        instance doesn't remember anything, it provides information for the task needed to perform
        correctly at the current moment.
        """
        self.state = state
        self.has_succeeded = False

    def start(self):
        """
        when this method is called, the task will be started.
        """
        raise NotImplementedError("not implemented")

    def next(self):
        """
        return the next task that it should be run.

        :return: a Task class object that can be instantiated
        """
        raise NotImplementedError("not implemented")
