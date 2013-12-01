#!/usr/bin/env python
import rospy
import milestone1
from std_msgs.msg import String

class SpeechRecognition():
    def __init__(self, is_listening = True):
        self.command = ""
        self.execute = False
        self.is_listening = is_listening
        rospy.Subscriber("recognizer/output", String, self.create_closure())

    def setListening(self, new_value):
        self.is_listening = new_value

    def create_closure(self):
        def callback(data):
            print ("Did you say %s" % data.data)
            self.command = data
        return callback

    def get_command(self):
        return self.command
