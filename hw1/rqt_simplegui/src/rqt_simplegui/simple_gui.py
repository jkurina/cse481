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
        
	button_box = QtGui.QHBoxLayout()
        button_box.addWidget(self.create_button('Say something',
		self.command_cb))
	button_box.addStretch(1)
        large_box.addLayout(button_box)
	large_box.addItem(QtGui.QSpacerItem(100,20))
	
	gripper = Gripper(Gripper.RIGHT, Gripper.OPEN)
	right_gripper = self.create_button('Right gripper!', gripper.create_closure())
        gripper = Gripper(Gripper.LEFT, Gripper.OPEN)
        left_gripper = self.create_button('Left gripper!', gripper.create_closure()) 
	large_box.addWidget(left_gripper)
        large_box.addWidget(right_gripper)
       
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
        if (button_name == 'Say something'):
            qWarning('Robot will say: something')
            self._sound_client.say('Soi soi soi!')
            
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

