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

class Navigation():
        def __init__(self):
            self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	    self.client.wait_for_server()

	# Navigates the PR2 to the location passed in
	def move_to_shelf(self, x, y):
	    goal = MoveBaseGoal()
	    goal.target_pose.header.frame_id = "map"
	    goal.target_pose.header.stamp = rospy.Time.now()
	    x = 1.54238247871
            y = -1.83420526981
            goal.target_pose.pose.position.x = x
	    goal.target_pose.pose.position.y = y
	    goal.target_pose.pose.orientation.z = 0.990642343157
	    goal.target_pose.pose.orientation.w = -0.136483507958
#middle book: 
#    x: 0.905410051346
#    y: -2.0606584549


	    # Send the goal to the action server
	    self.client.send_goal(goal)
	    rospy.loginfo("Sending navgoal to " + str(x) + ", " + str(y))

	    # Wait for the robot to navigate to the specified location
	    self.client.wait_for_result()
            #print (self.client.get_state())
 	    print ("Result: " + str(self.client.get_result()))
	    # Check if the navigation was successful
	    if (self.client.get_state() != GoalStatus.SUCCEEDED):
		rospy.loginfo("PR2's navigation was unsuccessful")
	    else:
		rospy.loginfo("PR2 navigated successfully")
		print ("State: " + str(self.client.get_state()))

