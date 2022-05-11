#!/usr/bin/env python3
"""
This node is responsible for creating GUI window that displays the current state of the robot
"""

# state constants
import os
import sys

import rospy
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QSize
from PyQt5.QtGui import QMovie
from topics import STATE_TOPIC
from std_msgs.msg import Int32

GO_TO_WAYPOINT = 1
ROTATION = 2
PICK_UP_TRASH = 3
GO_TO_COLLECTION_SITE = 4


class GUI(QtWidgets.QWidget):

    def __init__(self):
        super().__init__()
        rospy.init_node("gui")
        self.setGeometry(0, 0, 500, 500)
        self.label = QtWidgets.QLabel("Navigate to a waypoint")
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.addWidget(self.label)

        self.gif_path = os.path.split(os.path.abspath(__file__))[0]
        print(self.gif_path)

        self.movie = QMovie(os.path.join(self.gif_path, "/navigation.gif"))
        self.movie.setScaledSize(QSize(300, 225))
        self.label.setMovie(self.movie)
        self.movie.start()
        self.state_sub = rospy.Subscriber(STATE_TOPIC, Int32, self.create_state_cb())

    def create_state_cb(self):

        def cb(msg: Int32):
            if msg.data == GO_TO_WAYPOINT:
                movie = QMovie("navigation.gif")
                self.label.setText("Navigate to a waypoint")
            elif msg.data == ROTATION:
                movie = QMovie("gifs/rotation.gif")
                self.label.setText("Rotation to detect trash")
            elif msg.data == PICK_UP_TRASH:
                movie = QMovie("gifs/trash_pickup.gif")
                self.label.setText("Pick up trash")
            elif msg.data == GO_TO_COLLECTION_SITE:
                movie = QMovie("collection_site.gif")
                self.label.setText("Go to collection site")
            else:
                raise ValueError("invalid state")

            self.label.setMovie(movie)
            movie.start()

        return cb


if __name__ == '__main__':
    app = QtWidgets.QApplication([])
    widget = GUI()
    widget.show()

    while not rospy.is_shutdown():
        sys.exit(app.exec())
