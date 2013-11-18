import roslib
roslib.load_manifest('rospy')
import numpy
import rospy
from ar_track_alvar.msg import AlvarMarkers

class ReadMarkers():
    def __init__(self):
        self.marker_id = None
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.create_closure())
   
    def create_closure(self):
        def callback(data):
            if len(data.markers) is not 0:
                self.marker_id = data.markers[0].id
        return callback

    def get_marker_id(self):
        return self.marker_id

    def reset_marker_id(self):
        self.marker_id = None        
