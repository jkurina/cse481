import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('control_msgs')
import numpy
import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from move_base_msgs import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal

def move_to_shelf():
    client = SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    #goal.targest_pose.header.stamp = ??
    goal.targest_pose.pose.position.x = 1.0
    goal.targest_pose.pose.orientation.w = 1.0
    client.send_goal(goal)
    client.wait_for_result()
    print ("moved!")
