#!/usr/bin/env python

import roslib
roslib.load_manifest('sound_play')
roslib.load_manifest('rospy')

from subprocess import call
import rospy
from qt_gui.plugin import Plugin
from python_qt_binding import QtGui,QtCore
from python_qt_binding.QtGui import QWidget, QFrame
from python_qt_binding.QtGui import QGroupBox
from python_qt_binding.QtCore import QSignalMapper, qWarning, Signal
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from gripper import Gripper
from head import Head
from base import Base

class SimpleGUI(Plugin):

    sound_sig = Signal(SoundRequest)

    def __init__(self, context):
        super(SimpleGUI, self).__init__(context)
        self.setObjectName('SimpleGUI')
        self._widget = QWidget()
        
        self._sound_client = SoundClient()
        rospy.Subscriber('robotsound', SoundRequest, self.sound_cb)

        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
        self.sound_sig.connect(self.sound_sig_cb)
        
        large_box = QtGui.QVBoxLayout()
        
        #Sound textbox
        sound_textbox = QtGui.QLineEdit("Squirtle Squirtle") #Default Text
        sound_textbox.setFixedWidth(400)
        #Set a handle on the textbox to retrieve the text when button clicked
        self.sound_textbox = sound_textbox

        button_box = QtGui.QHBoxLayout()
        button_box.addWidget(self.create_button('Speak', self.command_cb))
    	button_box.addWidget(sound_textbox)
        button_box.addStretch(1)
        large_box.addLayout(button_box)
        large_box.addItem(QtGui.QSpacerItem(100,20))
        
        gripper = Gripper(Gripper.RIGHT, Gripper.OPEN)
        right_gripper = self.create_button('Right gripper!', gripper.create_closure())
        gripper = Gripper(Gripper.LEFT, Gripper.OPEN)
        left_gripper = self.create_button('Left gripper!', gripper.create_closure()) 
        large_box.addWidget(left_gripper)
        large_box.addWidget(right_gripper)

        up_head = Head(Head.UP)
        up_head_button = self.create_button('up head', up_head.create_closure())
        large_box.addWidget(up_head_button)
        down_head = Head(Head.DOWN)
        down_head_button = self.create_button('down head', down_head.create_closure())
        large_box.addWidget(down_head_button)
        right_head = Head(Head.RIGHT)
        right_head_button = self.create_button('right head', right_head.create_closure())
        large_box.addWidget(right_head_button)
        left_head = Head(Head.LEFT)
        left_head_button = self.create_button('left head', left_head.create_closure())
        large_box.addWidget(left_head_button)

        #forward
        forward_base = Base(Base.FORWARD)
        forward_base_button = self.create_button('move forward', forward_base.create_closure())
        large_box.addWidget(forward_base_button)
        #backward
        backward_base= Base(Base.BACKWARD)
      	backward_base_button = self.create_button('move backward', backward_base.create_closure())
        large_box.addWidget(backward_base_button)
        #left
        left_base= Base(Base.LEFT)
      	left_base_button = self.create_button('move left', left_base.create_closure())
        large_box.addWidget(left_base_button)
        #right
        right_base= Base(Base.RIGHT)
      	right_base_button= self.create_button('move right', right_base.create_closure())
        large_box.addWidget(right_base_button)
        #turn left
        turnleft_base= Base(Base.TURNLEFT)
      	turnleft_base_button = self.create_button('turn left', turnleft_base.create_closure())
        large_box.addWidget(turnleft_base_button)
        #turn right
        turnright_base= Base(Base.TURNRIGHT)
      	turnright_base_button = self.create_button('turn right', turnright_base.create_closure())
        large_box.addWidget(turnright_base_button)

        speech_box = QtGui.QHBoxLayout()
        self.speech_label = QtGui.QLabel('Robot has not spoken yet')
        palette = QtGui.QPalette()
        palette.setColor(QtGui.QPalette.Foreground,QtCore.Qt.blue)
        self.speech_label.setPalette(palette)
        speech_box.addWidget(self.speech_label)

        large_box.addLayout(speech_box)
        large_box.addStretch(1)

        self._widget.setObjectName('SimpleGUI')
        self._widget.setLayout(large_box)
        context.add_widget(self._widget)
        
    def sound_cb(self, sound_request):
        qWarning('Received sound.')
        self.sound_sig.emit(sound_request)
        
    def create_button(self, name, method):
        btn = QtGui.QPushButton(name, self._widget)
        btn.clicked.connect(method)
        return btn

    def sound_sig_cb(self, sound_request):
        qWarning('Received sound signal.')
        #if (sound_request.command == SoundRequest.SAY):
        qWarning('Robot said: ' + sound_request.arg)
        self.speech_label.setText('Robot said: ' + sound_request.arg)

    def command_cb(self):
        button_name = self._widget.sender().text()
        if (button_name == 'Speak'):
            qWarning('Robot will say: ' + self.sound_textbox.text())
            self._sound_client.say(self.sound_textbox.text())
            
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

