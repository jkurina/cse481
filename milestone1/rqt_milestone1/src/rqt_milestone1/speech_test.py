import rospy
import milestone1
from speech_recognition import SpeechRecognition
from std_msgs.msg import String

class SpeechTest:
    def put_back(self):
        print ("please give me a book")

    if __name__ == '__main__':
        speech_recognition = SpeechRecognition()
        speech_recognition.is_listening = True
        command = speech_recognition.get_command()
        rospy.loginfo("current command is: " + command)
        if command is None:
            print ("How may I help you?")
        if command is "PUT-THIS-BOOK-BACK":
            self.put_back()
