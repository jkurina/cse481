#!/usr/bin/env python

import roslib
roslib.load_manifest('sound_play')
roslib.load_manifest('rospy')
roslib.load_manifest('visualization_msgs')
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from subprocess import call
import rospy
from qt_gui.plugin import Plugin
from python_qt_binding import QtGui,QtCore
from python_qt_binding.QtGui import QWidget, QFrame
from python_qt_binding.QtGui import QGroupBox
from python_qt_binding.QtCore import QSignalMapper, qWarning, Signal
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from navigation import move_to_shelf
import os

class Milestone1GUI(Plugin):

    sound_sig = Signal(SoundRequest)

    def __init__(self, context):
        super(Milestone1GUI, self).__init__(context)
        self._widget = QWidget()
        self._widget.setFixedSize(600, 600)
        self._sound_client = SoundClient()
        rospy.Subscriber('robotsound', SoundRequest, self.sound_cb)

        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
        
        large_box = QtGui.QVBoxLayout()

        # Button to move to shelf
        nav_button = self.create_button('navigate', move_to_shelf)
        
        self._widget.setObjectName('Milestone1GUI')
        self._widget.setLayout(large_box)
        context.add_widget(self._widget)
        self._widget.setStyleSheet("QWidget { image: url(%s) }" %
                (str(os.path.dirname(os.path.realpath(__file__))) +
                "/../../rosie_background.jpg"))

    def sound_cb(self, sound_request):
        qWarning('Received sound.')
        self.sound_sig.emit(sound_request)
        
    def create_button(self, name, method):
        btn = QtGui.QPushButton(name, self._widget)
        btn.clicked.connect(method)
        btn.setAutoRepeat(True)
        return btn

    def command_cb(self):
        button_name = self._widget.sender().text()
        if (button_name == 'Speak'):
            qWarning('Robot will say: ' + self.sound_textbox.text())
            self._sound_client.say(self.sound_textbox.text())
            
    
