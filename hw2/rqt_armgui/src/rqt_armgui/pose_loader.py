#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('trajectory_msgs')
roslib.load_manifest('control_msgs')
roslib.load_manifest('pr2_mechanism_msgs')
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('actionlib')

from subprocess import call
import threading
import rospy
from qt_gui.plugin import Plugin
from python_qt_binding import QtGui,QtCore
from python_qt_binding.QtGui import QWidget, QFrame
from python_qt_binding.QtGui import QGroupBox, QComboBox
from python_qt_binding.QtCore import QSignalMapper, qWarning, Signal
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryGoal
from control_msgs.msg import JointTrajectoryAction
from pr2_mechanism_msgs.srv import SwitchController
from sensor_msgs.msg import JointState
from actionlib import SimpleActionClient

class PoseSaverDialog(QtGui.QDialog):
    def __init__(self, parent=None):
        # TODO: init
        QtGui.QDialog.__init__(self, parent)
        self.setObjectName('PoseSaver')

        large_box = QtGui.QVBoxLayout()
        # add things here

        # Textbox to enter words for PR2 to say 
        default_text = "New pose 1"  # TODO: increment the number!
        name_textbox = QtGui.QLineEdit(default_text)
        name_textbox.setFixedWidth(450)
        
        # Set a handler on the textbox to retrieve the text when button clicked
        self.name_textbox = name_textbox

        button_box = QtGui.QHBoxLayout()
        button_box.addItem(QtGui.QSpacerItem(15,20))
        save_button = self.create_button('Save!', self.save_pose)
        save_button.setDefault(True)
        button_box.addWidget(save_button)
        button_box.addWidget(name_textbox)
        button_box.addStretch(1)
        large_box.addLayout(button_box)

        self.setLayout(large_box)

    # TODO: GUI stuff!
    def create_button(self, name, method):
        btn = QtGui.QPushButton(name, self)
        btn.clicked.connect(method)
        # btn.setAutoRepeat(True)
        return btn

    def save_pose(self):
        qWarning('Saving pose as: ' + self.name_textbox.text())
        # TODO
        self.accept()

class PoseLoader():
    LEFT = 'l'
    RIGHT = 'r'

    def __init__(self, side, gui):
        assert(side == PoseLoader.LEFT or side == PoseLoader.RIGHT)
        self.side = side
        self.gui = gui
        self.data = ['foo', 'bar', 'baz']

    def label(self):
        if self.side == PoseLoader.LEFT:
          return "Left arm pose loader!"
        else: # self.side == PoseLoader.RIGHT:
          return "Right arm pose loader!"

    def load_pose(self, index):
        qWarning('Selected pose ' + index)

    def create_button(self):
        combo_box = QComboBox()
        for item in self.data:
            combo_box.addItem(item, "userdata")
        combo_box.activated.connect(self.load_pose)
        return combo_box


