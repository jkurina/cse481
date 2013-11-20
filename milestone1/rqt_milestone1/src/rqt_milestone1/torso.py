#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('control_msgs')
roslib.load_manifest('pr2_controllers_msgs')
import rospy
from pr2_controllers_msgs.msg import SingleJointPositionAction
from pr2_controllers_msgs.msg import SingleJointPositionGoal
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus

class Torso:
    
    def __init__(self):
        name_space = '/torso_controller/position_joint_action'
        self.torso_client = SimpleActionClient(name_space, SingleJointPositionAction)
        self.torso_client.wait_for_server()

    def move(self, position):
        assert(position >= 0.0 and position <= 0.2)
        self.position = position
        
        up = SingleJointPositionGoal()
        up.position = self.position
        up.min_duration = rospy.Duration(2.0)
        up.max_velocity = 1.0

        self.torso_client.send_goal(up)
        self.torso_client.wait_for_result()

