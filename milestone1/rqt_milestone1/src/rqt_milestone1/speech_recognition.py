#!/usr/bin/env python
import rospy
import milestone1
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class SpeechRecognition():
    def __init__(self, milestone1, is_listening = True):
        self.command = None
        self.execute = False
        self.milestone1 = milestone1
        self.is_listening = is_listening
        self.looking_for_a_book = False
        rospy.Subscriber("recognizer/output", String, self.create_closure())

    def setListening(self, new_value):
        self.is_listening = new_value

    def create_closure(self):
        def callback(data):
            print ("Received command: %s" % data.data)
            if not self.is_listening:
                print ("Lalalala I'm not listening")
                return
            if self.looking_for_a_book:
                self.looking_for_a_book = False
                self.is_listening = False
                print ("Picking up book %s" % data.data)
                self.command(data.data)
                self.command = None
                self.is_listening = True
                return
            if self.command is None:
                self.command = self.milestone1.get_function(data.data)
                if self.command is None:
                    self.say("I don't understand how to %s" % data.data)
                elif self.command is "BRING-ME-A-BOOK":
                    self.say("What book would you like?")
                    self.looking_for_a_book = True
                else:
                    self.say("Do you want me to %s?" % data.data)
            else:
                if data is not "YES":
                    print ("Cancelling command.")
                    self.command = None
                else:
                    self.say("Executing command!")
                    self.is_listening = False
                    self.command()
                    self.command = None
                    self.is_listening = True

        return callback

    def get_command(self):
        return self.command

    def say(self, text):
        self.milestone1._sound_client.say(text)
