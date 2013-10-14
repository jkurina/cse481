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

class Gripper():
    
    LEFT = 'l'
    RIGHT = 'r'
    OPEN = 1
    CLOSED = 0

    def __init__(self, direction, state):
        assert(direction == Gripper.LEFT or direction == Gripper.RIGHT)
        assert(state == Gripper.OPEN or state == Gripper.CLOSED)
        self.direction = direction
        self.state = state
         
    
    def create_closure(self):
        
        def move_gripper():
            name_space = '/{0}_gripper_controller/gripper_action'.format(
			self.direction)
            gripper_client = SimpleActionClient(name_space,
			GripperCommandAction)
            gripper_client.wait_for_server()

            gripper_goal = GripperCommandGoal()
            
	    if self.state:
                # The gripper is open. Close it and update its state.
                gripper_goal.command.position = 0.0
                self.state = Gripper.CLOSED
            else:
                # The gripper is closed. Open it and update its state.
                gripper_goal.command.position = 0.08
                self.state = Gripper.OPEN

            gripper_goal.command.max_effort = 30.0

            gripper_client.send_goal(gripper_goal)
            gripper_client.wait_for_result(rospy.Duration(10.0))
               
	return move_gripper

 

