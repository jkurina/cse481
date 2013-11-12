#!/usr/bin/env python
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('control_msgs')
roslib.load_manifest('pr2_controllers_msgs')
import rospy
from control_msgs.msg import GripperCommandAction
from control_msgs.msg import GripperCommandGoal
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
import time

class Gripper():
    
    LEFT = 'l'
    RIGHT = 'r'

    def __init__(self, direction):
        assert(direction == Gripper.LEFT or direction == Gripper.RIGHT)
        self.direction = direction
        name_space = '/{0}_gripper_controller/gripper_action'.format(
        self.direction)
        self.gripper_client = SimpleActionClient(name_space,
        GripperCommandAction)
        self.gripper_client.wait_for_server()
        
    def open_gripper(self, wait = False):
        self.change_state(0.08, wait)

    def close_gripper(self, wait = False):
        self.change_state(0.0, wait)

    def change_state(self, value, wait):
        gripper_goal = GripperCommandGoal()
        gripper_goal.command.position = value
        gripper_goal.command.max_effort = 30.0
        self.gripper_client.send_goal(gripper_goal)
        if wait:
            self.gripper_client.wait_for_result()
            if not self.gripper_client.get_result().reached_goal:
                time.sleep(9)
