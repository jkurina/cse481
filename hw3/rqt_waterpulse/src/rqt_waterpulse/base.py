#!/usr/bin/env python
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import time
import waterpulse

class Base():

    FORWARD = 'f'
    BACKWARD = 'b'
    LEFT = 'l'
    RIGHT = 'r'
    CLOCKWISE = 'cl'
    COUNTER = 'cr'
    
    def __init__(self, direction, gui):   
        assert(direction == Base.FORWARD or 
               direction == Base.BACKWARD or
               direction == Base.LEFT or
               direction == Base.RIGHT or
               direction == Base.CLOCKWISE or
               direction == Base.COUNTER)
        self.direction = direction
        self.gui = gui

    def create_closure(self):
        
        def move_base():
            
            topic_name = '/base_controller/command'
            base_publisher = rospy.Publisher(topic_name, Twist)

            twist_msg = Twist()
            if(self.direction == Base.FORWARD):
                twist_msg.linear = Vector3(0.5, 0.0, 0.0)
                twist_msg.angular = Vector3(0.0, 0.0, 0.0)
            elif(self.direction == Base.BACKWARD):
                twist_msg.linear = Vector3(-0.5, 0.0, 0.0)
                twist_msg.angular = Vector3(0.0, 0.0, 0.0)
            elif(self.direction == Base.LEFT):
                twist_msg.linear = Vector3(0.0, 0.5, 0.0)
                twist_msg.angular = Vector3(0.0, 0.0, 0.0)
            elif(self.direction == Base.RIGHT):
                twist_msg.linear = Vector3(0.0, -0.5, 0.0)
                twist_msg.angular = Vector3(0.0, 0.0, 0.0)
            elif(self.direction == Base.COUNTER):
                twist_msg.linear = Vector3(0.0, 0.0, 0.0)
                twist_msg.angular = Vector3(0.0, 0.0, 0.5)
            elif(self.direction == Base.CLOCKWISE):
                twist_msg.linear = Vector3(0.0, 0.0, 0.0)
                twist_msg.angular = Vector3(0.0, 0.0, -0.5)

            base_publisher.publish(twist_msg)
            self.gui.show_text_in_rviz("Base!")

        return move_base
