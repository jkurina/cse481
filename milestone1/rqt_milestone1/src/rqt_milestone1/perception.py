import roslib
roslib.load_manifest('rospy')
import numpy
import rospy

def callback(data):
    rospy.loginfo("heard: " + data.data)

def listener():
    rospy.Subscriber("ar_pose_marker", ar_track_alvar/AlvarMarkers, callback)
