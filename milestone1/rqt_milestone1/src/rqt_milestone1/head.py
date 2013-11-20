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
    speed = 1

    def __init__(self, direction, gui):
        assert(direction == Head.LEFT or direction == Head.RIGHT
            or direction == Head.UP or direction == Head.DOWN)
        self.direction = direction
        self.gui = gui

    def get_frame(self):
        return 'head_tilt_link'

    def get_target(self):
        point = Point(0, 0, 0)
        head_speed = Head.speed * 0.1
        if (self.direction == Head.LEFT):
            point = Point(1, head_speed, 0)
        elif (self.direction == Head.RIGHT):
            point = Point(1, -head_speed, 0)
        elif (self.direction == Head.UP):
            point = Point(1, 0, head_speed)
        elif (self.direction == Head.DOWN):  # Head.DOWN
            point = Point(1, 0, -head_speed)
        print ("point is")
        print (point)
        return point

    def create_closure(self):
        def move_head():
            name_space = '/head_traj_controller/point_head_action'

            head_client = SimpleActionClient(name_space, PointHeadAction)
            head_client.wait_for_server()

            head_goal = PointHeadGoal()
            head_goal.target.header.frame_id = self.get_frame()
            head_goal.min_duration = rospy.Duration(0.3)
            head_goal.target.point = self.get_target()
            head_goal.max_velocity = 10.0
            head_client.send_goal(head_goal)
            head_client.wait_for_result(rospy.Duration(2.0))

            if (head_client.get_state() != GoalStatus.SUCCEEDED):
                rospy.logwarn('Head action unsuccessful.')
            
            self.gui.show_text_in_rviz("Head!")
        return move_head

    def tilt_head(self, down=True):
        name_space = '/head_traj_controller/point_head_action'

        head_client = SimpleActionClient(name_space, PointHeadAction)
        head_client.wait_for_server()

        head_goal = PointHeadGoal()
        head_goal.target.header.frame_id = self.get_frame()
        head_goal.min_duration = rospy.Duration(0.3)
        if down:
            head_goal.target.point = Point(1, 0, Head.speed * -0.1)
        else:
            head_goal.target.point = Point(1, 0, Head.speed * 0.1)
        head_goal.max_velocity = 10.0
        head_client.send_goal(head_goal)
        head_client.wait_for_result(rospy.Duration(2.0))

        if (head_client.get_state() != GoalStatus.SUCCEEDED):
            rospy.logwarn('Head action unsuccessful.')


    @staticmethod
    def set_speed(value):
        Head.speed = value

