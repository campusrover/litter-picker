#!/usr/bin/env python3
"""
This node is responsible for creating GUI window that displays the current state of the robot
"""

# state constants
import rospy
from PyQt5 import QtCore, QtGui, QtWidgets
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
        self.state_sub = rospy.Subscriber(STATE_TOPIC, Int32, self.create_state_cb())
        self.layout = QtWidgets.QVBoxLayout(self)
        self.label = QtWidgets.QLabel("GO TO THE NEXT WAYPOINT")
        self.layout.addWidget(self.label)

    def create_state_cb(self):

        def cb(msg: Int32):
            if msg.data == GO_TO_WAYPOINT:
                movie = QMovie("../gifs/navigation.gif")
                self.label.setText("Navigate to a waypoint")
            elif msg.data == ROTATION:
                movie = QMovie("../gifs/rotation.gif")
                self.label.setText("Rotation to detect trash")
            elif msg.data == PICK_UP_TRASH:
                movie = QMovie("../gifs/trash_pickup.gif")
                self.label.setText("Pick up trash")
            elif msg.data == GO_TO_COLLECTION_SITE:
                movie = QMovie("../gifs/collection_site.gif")
                self.label.setText("Go to collection site")
            else:
                raise ValueError("invalid state")

            self.label.setMovie(movie)

        return cb
