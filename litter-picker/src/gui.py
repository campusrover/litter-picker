#!/usr/bin/env python3
import sys

from PyQt5 import QtCore, QtGui, QtWidgets
from std_msgs.msg import String
import rospy


class StatusWindow(QtWidgets.QWidget):

    def __init__(self):
        super().__init__()
        rospy.init_node("gui")
        self.state = QtWidgets.QLabel("Waiting for the master to be initialized",
                                      alignment=QtCore.Qt.AlignCenter)
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.addWidget(self.state)
        self.status_sub = rospy.Subscriber('master/state', String, self.update_state_callback())

    def update_state_callback(self):
        def cb(msg: String):
            self.state.setText(msg.data)

        return cb


if __name__ == "__main__":
    app = QtWidgets.QApplication([])
    widget = StatusWindow()
    widget.show()

    while not rospy.is_shutdown():
        sys.exit(app.exec())
