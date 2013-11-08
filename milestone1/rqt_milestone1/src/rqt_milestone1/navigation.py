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
# TODO imports

# broken :(((((
def move_to_shelfz():
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

def move_to_shelf():
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped)
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = 0.380967617035
    msg.pose.position.y = -1.6217675209
    msg.pose.orientation.z = 0.953879350922
    msg.pose.orientation.w = -0.300190246152
    pub.publish(msg)
    print("moved!")
