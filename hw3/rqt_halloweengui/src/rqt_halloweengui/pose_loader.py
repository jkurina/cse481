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
from arm_db import ArmDB

class PoseLoader():
    LEFT = 'l'
    RIGHT = 'r'
    
    def __init__(self, side, gui):
        assert(side == PoseLoader.LEFT or side == PoseLoader.RIGHT)
        self.side = side
        self.gui = gui
        self.arm_db = ArmDB()

    def label(self):
        if self.side == PoseLoader.LEFT:
          return "Left arm pose loader!"
        else: # self.side == PoseLoader.RIGHT:
          return "Right arm pose loader!"

    def load_pose(self, index):
        qWarning('Selected pose ' + index)

    def create_button(self):
        # self.data is a dictionary name-->array[positions]
        if self.side == PoseLoader.LEFT:
            self.data = self.arm_db.getAllLeftPos()
        else: # self.side == PoseLoader.RIGHT:
            self.data = self.arm_db.getAllRightPos()
        
        combo_box = QComboBox()
        for key in self.data.keys():
            combo_box.addItem(key, self.data.get(key))
        combo_box.activated.connect(self.load_pose)
        return combo_box
