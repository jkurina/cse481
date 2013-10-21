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
from python_qt_binding.QtGui import QGroupBox
from python_qt_binding.QtCore import QSignalMapper, qWarning, Signal
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryGoal
from control_msgs.msg import JointTrajectoryAction
from pr2_mechanism_msgs.srv import SwitchController
from sensor_msgs.msg import JointState
from actionlib import SimpleActionClient

from arm_db import ArmDB

class PoseSaverDialog(QtGui.QDialog):
    def __init__(self, side, gui, parent=None):
        self.side = side
        self.gui = gui

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
        # Add to database
        name = self.name_textbox.text()
        pose = self.gui.get_joint_state(self.side)
        qWarning('Saving pose as: ' + name)
        isAdded = self.gui.arm_db.savePos(self.side, name, pose)

        if not isAdded:
            # name already exists...
            self.reject()
            return

        # Update GUI
        if self.side == PoseSaver.LEFT:
            self.gui.combo_box_left.addItem(name, pose)  # is this the right data to add?
        else: # self.side == PoseSaver.RIGHT:
            self.gui.combo_box_right.addItem(name, pose)
        self.accept()

class PoseSaver():
    LEFT = 'l'
    RIGHT = 'r'

    def __init__(self, side, gui):
        assert(side == PoseSaver.LEFT or side == PoseSaver.RIGHT)
        self.side = side
        self.gui = gui

    def label(self):
        if self.side == PoseSaver.LEFT:
          return "Left arm pose saver!"
        else: # self.side == PoseSaver.RIGHT:
          return "Right arm pose saver!"

    def create_closure(self):
        def launch_popup():
          self.dialog = PoseSaverDialog(self.side, self.gui)
          self.dialog.show()

        return launch_popup

