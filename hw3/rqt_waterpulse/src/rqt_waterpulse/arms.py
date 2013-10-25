
#!/usr/bin/env python
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('control_msgs')
import rospy
from control_msgs.msg import PointHeadAction
from control_msgs.msg import PointHeadGoal
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from python_qt_binding import QtGui,QtCore
import watergun

class PoseSelectionDialog(QtGui.QDialog):
    def __init__(self, parent=None):
      # TODO: init
      QtGui.QDialog.__init__(self, parent)

    # TODO: GUI stuff!

class Arm():
    LEFT = 'l'
    RIGHT = 'r'

    def __init__(self, side, gui):
        assert(side == Arm.LEFT or side == Arm.RIGHT)
        self.side = side
        self.gui = gui

    def label(self):
        if self.side == Arm.LEFT:
          return "Left arm!"
        else: # self.side == Arm.RIGHT:
          return "Right arm!"

    def create_closure(self):
        def launch_popup():
          self.dialog = PoseSelectionDialog()
          self.dialog.show()

        return launch_popup

