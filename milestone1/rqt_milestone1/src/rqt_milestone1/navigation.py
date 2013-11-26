import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('actionlib')
roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('control_msgs')
import numpy
import rospy
import actionlib
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Navigates the PR2 to the location passed in
def move_to_shelf(x, y):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = 1.353879350922
    goal.target_pose.pose.orientation.w = -0.300190246152
    
    # Send the goal to the action server
    client.send_goal(goal)
    rospy.loginfo("Sending navgoal to " + str(x) + ", " + str(y))

    # Wait for the robot to navigate to the specified location
    client.wait_for_result()
    
    # Check if the navigation was successful
    if (client.get_state() != GOalStatus.SUCCEEDED):
        rospy.loginfo("PR2's navigation was unsuccessful")
    else:
        rospy.loginfo("PR2 navigated successfully")


