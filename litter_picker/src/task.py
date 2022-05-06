import rospy

from master import LitterPickerState


class Task:
    """
    An abstract class that represents a single task to be executed at a certain point. The subclass
    should implement the start() method, which begins the execution of the task, as well as the next()
    method which returns the next task to be executed give certain condition passed into self.state
    """
    def __init__(self, state: LitterPickerState):
        """
        constructor fo the task

        :param state: this represents the state of the previous task execution, as the task
        instance doesn't remember anything, it provides information for the task needed to perform
        correctly at the current moment.
        """

        # the current litter picker state/the state after the last task has been executed
        self.state = state

        # whether the task was executed successfully or not
        self.has_succeeded = False

        # the default rate to sleep in the while loop that continue publishes message to prevent
        # the ros node from overheating.
        self.rate = rospy.Rate(10)

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
