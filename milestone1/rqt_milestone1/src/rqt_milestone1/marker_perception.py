import roslib
roslib.load_manifest('rospy')
import numpy
import rospy
from ar_track_alvar.msg import AlvarMarkers

class ReadMarkers():
    def __init__(self, is_listening = True):
        self.marker_id = 11
        self.is_listening = is_listening
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.create_closure())

    def setListening(self, new_value):
        self.is_listening = new_value
   
    def create_closure(self):
        def callback(data):
            if len(data.markers) is not 0 and self.is_listening:
                self.marker_id = data.markers[0].id
                print (str(self.marker_id))
        return callback

    def get_marker_id(self):
        return 14

    def reset_marker_id(self):
        self.marker_id = None        
