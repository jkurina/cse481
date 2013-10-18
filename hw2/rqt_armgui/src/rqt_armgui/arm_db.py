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
import json

class ArmDB:

    instance = None

    class Singleton:

        positions = None
        
        def __init__(self):
            pass
            #if self.positions == None:
            #    self.loadPos()

        def loadPos(self):
            pass
            #f = open('arm_pos.db', 'w+')
            #if f:
            #    positions = json.loads(f.read())

        def savePos(self, side_prefix, name, positions):
            return 1

        def rmPos(self, side_prefix, name, positions):
            return 1

        def loadLeftPos(self):
            return []

        def loadRightPos(self):
            return []
          
    def __init__(self):
        if ArmDB.instance is None:
            ArmDB.instance = ArmDB.Singleton()

    #eg, savePos('r', 'pos name', get_joint_state())
    #get_join_state() is the one from arm_gui.py
    def savePos(self, side_prefix, name, positions):
        ArmDB.instance.savePos(side_prefix, name, positions)

    def rmPos(self, side_prefix, name, positions):
        ArmDB.instance.rmPos(side_prefix, name, positions)

    #return an array
    def loadLeftPos(self):
        return ArmDB.instance.loadLeftPos()

    def loadRightPos(self):
        return ArmDB.instance.loadRightPos();
