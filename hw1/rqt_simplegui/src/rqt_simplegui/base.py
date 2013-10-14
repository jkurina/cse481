#!/usr/bin/env python
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import time

class Base():

    FORWARD = 'f'
    BACKWARD = 'b'
    LEFT = 'l'
    RIGHT = 'r'
    TURNLEFT = 'tl'
    TURNRIGHT = 'tr'
    
    def __init__(self, direction):   
        assert(direction == Base.FORWARD or 
               direction == Base.BACKWARD or
               direction == Base.LEFT or
               direction == Base.RIGHT or
               direction == Base.TURNLEFT or
               direction == Base.TURNRIGHT)
        self.direction = direction

    def create_closure(self):
        
        def move_base():
            
            topic_name = '/base_controller/command'
            base_publisher = rospy.Publisher(topic_name, Twist)

            twist_msg = Twist()
            if(self.direction == Base.FORWARD):
                twist_msg.linear = Vector3(1.0, 0.0, 0.0)
                twist_msg.angular = Vector3(0.0, 0.0, 0.0)
            elif(self.direction == Base.BACKWARD):
                twist_msg.linear = Vector3(-100.0, 0.0, 0.0)
                twist_msg.angular = Vector3(0.0, 0.0, 0.0)
            elif(self.direction == Base.TURNLEFT):
                print "turning left"
                twist_msg.linear = Vector3(0.0, 0.0, 0.0)
                twist_msg.angular = Vector3(0.0, 0.0, -10.0);

            
            for i in range(100):
                base_publisher.publish(twist_msg)
                time.sleep(0.1)


        return move_base
