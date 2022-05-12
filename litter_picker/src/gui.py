#!/usr/bin/env python3
"""
This node is responsible for creating GUI window that displays the current state of the robot
"""

# state constants
import os
import sys

import rospy
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QMovie
from topics import STATE_TOPIC
from std_msgs.msg import Int32
from utils import get_gif_path

GO_TO_WAYPOINT = 1
ROTATION = 2
PICK_UP_TRASH = 3
GO_TO_COLLECTION_SITE = 4


class GUI(QtWidgets.QWidget):

    def __init__(self):
        super().__init__()
        rospy.init_node("gui")
        self.label = QtWidgets.QLabel()
        self.text_label = QtWidgets.QLabel("Navigate to a waypoint")
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.addWidget(self.label)
        self.layout.addWidget(self.text_label)

        self.gif_path = os.path.split(os.path.abspath(__file__))[0]

        self.movie = QMovie(get_gif_path("navigation.gif"))
        self.label.setMovie(self.movie)
        self.movie.start()
        self.state_sub = rospy.Subscriber(STATE_TOPIC, Int32, self.create_state_cb())

        self.setAttribute(Qt.WA_DeleteOnClose)

    def create_state_cb(self):

        def cb(msg: Int32):
            if msg.data == GO_TO_WAYPOINT:
                self.movie = QMovie(get_gif_path("navigation.gif"))
                self.text_label.setText("Navigate to a waypoint")
            elif msg.data == ROTATION:
                self.movie = QMovie(get_gif_path("rotation.gif"))
                self.text_label.setText("Rotation to detect trash")
            elif msg.data == PICK_UP_TRASH:
                self.movie = QMovie(get_gif_path("trash_pickup.gif"))
                self.text_label.setText("Pick up trash")
            elif msg.data == GO_TO_COLLECTION_SITE:
                self.movie = QMovie(get_gif_path("collection_site.gif"))
                self.text_label.setText("Go to collection site")
            else:
                raise ValueError("invalid state")
            self.label.setMovie(self.movie)
            self.label.movie().start()

        return cb


if __name__ == '__main__':
    app = QtWidgets.QApplication([])
    widget = GUI()
    widget.show()
    rate = rospy.Rate(10)

    sys.exit(app.exec())
