import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('control_msgs')
import numpy
import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped

# Navigates the PR2 to the location of the bookshelf
def move_to_shelf():
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped)
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = 0.108998298645
    msg.pose.position.y = -2.0217675209
    msg.pose.orientation.z = 1.353879350922
    msg.pose.orientation.w = -0.300190246152
    pub.publish(msg)
    rospy.loginfo("Navigated to the bookshelf")

# book id 14
#    msg.pose.position.x = 0.780967617035
#    msg.pose.position.y = -2.0217675209
#    msg.pose.orientation.z = 1.353879350922
#    msg.pose.orientation.w = -0.300190246152

