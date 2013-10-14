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

class Head():
    LEFT = 'l'
    RIGHT = 'r'
    UP = 'u'
    DOWN = 'd'

    def __init__(self, direction):
        assert(direction == Head.LEFT or direction == Head.RIGHT
            or direction == Head.UP or direction == Head.DOWN)
        self.direction = direction

    def get_target(self):
        # TODO: figure out what this means.
        return Point(1, 1, 1.35)

    def create_closure(self):
        def move_head():
            name_space = '/head_traj_controller/point_head_action'

            head_client = SimpleActionClient(name_space, PointHeadAction)
            head_client.wait_for_server()

            head_goal = PointHeadGoal()
            head_goal.target.header.frame_id = 'base_link'
            head_goal.min_duration = rospy.Duration(1.0)
            head_goal.target.point = self.get_target()
            head_client.send_goal(head_goal)
            head_client.wait_for_result(rospy.Duration(10.0)) # check this

            if (head_client.get_state() != GoalStatus.SUCCEEDED):
                rospy.logwarn('Head action unsuccessful.')
            else:
                print("moved the head!") # TODO remove this
        return move_head

