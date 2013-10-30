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
from gripper import Gripper
from head import Head
from base import Base
from control_msgs.msg import JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction
from actionlib import SimpleActionClient
import os
import time

class WaterPulse(Plugin):

    sound_sig = Signal(SoundRequest)
    positions = [[-0.013001995432544766, -0.24895825213211362, -0.0055992576345036404, -1.7647281786034612, -496.3549908032631, -0.09960750521541717, 25.125128627428623], [0.0029160750999965845, -0.29700816845544387, 0.010917289481594317, -0.8034506550168371, -496.35799885178454, -0.09965101413551591, 25.12517213634872]]
 

    def __init__(self, context):
        super(WaterPulse, self).__init__(context)
        self.setObjectName('WaterPulse')
        self._widget = QWidget()
        self._widget.setFixedSize(600, 600)
        self._sound_client = SoundClient()
        rospy.Subscriber('robotsound', SoundRequest, self.sound_cb)

        self.l_joint_names = ['l_shoulder_pan_joint',
                              'l_shoulder_lift_joint',
                              'l_upper_arm_roll_joint',
                              'l_elbow_flex_joint',
                              'l_forearm_roll_joint',
                              'l_wrist_flex_joint',
                              'l_wrist_roll_joint']
 
        l_traj_contoller_name = None
        l_traj_controller_name = '/l_arm_controller/joint_trajectory_action'
	self.l_traj_action_client = SimpleActionClient(l_traj_controller_name,
		JointTrajectoryAction)
	rospy.loginfo('Waiting for a response from the trajectory '
		+ 'action server for LEFT arm...')
	self.l_traj_action_client.wait_for_server()

         
        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
        self.sound_sig.connect(self.sound_sig_cb)
        
        large_box = QtGui.QVBoxLayout()

        # Textbox to enter words for PR2 to say 
        sound_textbox = QtGui.QLineEdit("Squirtle Squirtle")  # Default Text
        sound_textbox.setFixedWidth(450)
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker)
        
        # Set a handler on the textbox to retrieve the text when button clicked
        self.sound_textbox = sound_textbox

        button_box = QtGui.QHBoxLayout()
        button_box.addItem(QtGui.QSpacerItem(15,20))
        button_box.addWidget(self.create_button('Speak', self.command_cb))
        button_box.addWidget(sound_textbox)
        button_box.addStretch(1)
        large_box.addLayout(button_box)
        
        # 'Trick or Treat' & 'Thank You' Buttons
        halloween_box = QtGui.QHBoxLayout()
        halloween_box.addItem(QtGui.QSpacerItem(15,20))
        halloween_box.addWidget(self.create_button('Trick or Treat', self.command_cb))
        halloween_box.addWidget(self.create_button('Thank You', self.command_cb))
        halloween_box.addStretch(1)
        large_box.addLayout(halloween_box)
        
        # Buttons to move the PR2's head
        up_head = Head(Head.UP, self)
        head_box = QtGui.QVBoxLayout()
        up_head_box = QtGui.QHBoxLayout()
        up_head_button = self.create_button('^', up_head.create_closure())
        down_head = Head(Head.DOWN, self)
        down_head_box = QtGui.QHBoxLayout()
        down_head_button = self.create_button('v', down_head.create_closure())
        right_head = Head(Head.RIGHT, self)
        right_head_button = self.create_button('>', right_head.create_closure())
        left_head = Head(Head.LEFT, self)
        left_head_button = self.create_button('<', left_head.create_closure())
        left_right_head_box = QtGui.QHBoxLayout()

        head_speed_sld = QtGui.QSlider(QtCore.Qt.Horizontal)
        head_speed_sld.setFocusPolicy(QtCore.Qt.NoFocus)
        head_speed_sld.setMinimum(1)
        head_speed_sld.setMaximum(5)
        head_speed_sld.valueChanged[int].connect(Head.set_speed)

        up_head_box.addItem(QtGui.QSpacerItem(235,20))
        up_head_box.addWidget(up_head_button)
        up_head_box.addItem(QtGui.QSpacerItem(275,20))
        left_right_head_box.addItem(QtGui.QSpacerItem(160,20))
        left_right_head_box.addWidget(left_head_button)
        left_right_head_box.addItem(QtGui.QSpacerItem(60,20))
        left_right_head_box.addWidget(right_head_button)
        left_right_head_box.addItem(QtGui.QSpacerItem(225,20))
        down_head_box.addItem(QtGui.QSpacerItem(235,20))
        down_head_box.addWidget(down_head_button)
        down_head_box.addItem(QtGui.QSpacerItem(275,20))
        head_sld_box = QtGui.QHBoxLayout()
        head_sld_box.addItem(QtGui.QSpacerItem(225,20))
        head_sld_box.addWidget(head_speed_sld)
        head_sld_box.addItem(QtGui.QSpacerItem(225,20))
        head_box.addLayout(up_head_box)
        head_box.addLayout(left_right_head_box)
        head_box.addLayout(down_head_box)
        head_box.addLayout(head_sld_box)
        large_box.addLayout(head_box)        

        # Buttons to move the grippers
        gripper = Gripper(Gripper.RIGHT, Gripper.OPEN, self)
        right_gripper = self.create_button('Right gripper',
                gripper.create_closure())
        gripper = Gripper(Gripper.LEFT, Gripper.OPEN, self)
        left_gripper = self.create_button('Left gripper', gripper.create_closure()) 
        knock_button = self.create_button('Knock', self.knock)
        large_box.addItem(QtGui.QSpacerItem(100,250))
        gripper_box = QtGui.QHBoxLayout()
        gripper_box.addItem(QtGui.QSpacerItem(75,20))
        gripper_box.addWidget(left_gripper)
        gripper_box.addWidget(knock_button)
        gripper_box.addItem(QtGui.QSpacerItem(450,20))
        gripper_box.addWidget(right_gripper)
        gripper_box.addItem(QtGui.QSpacerItem(75,20))
        large_box.addLayout(gripper_box)

        # Buttons to move the base
        base_box = QtGui.QVBoxLayout()
        large_box.addItem(QtGui.QSpacerItem(100,100))
        forward_base_box = QtGui.QHBoxLayout()
        forward_base = Base(Base.FORWARD, self)
        forward_base_button = self.create_button('move forward', forward_base.create_closure())
        forward_base_box.addItem(QtGui.QSpacerItem(400,20))
        forward_base_box.addWidget(forward_base_button)
        forward_base_box.addItem(QtGui.QSpacerItem(400,20))
        base_box.addLayout(forward_base_box)

        left_right_base_box = QtGui.QHBoxLayout()
        left_base= Base(Base.LEFT, self)
        left_base_button = self.create_button('move left',
                left_base.create_closure())

        right_base= Base(Base.RIGHT, self)
        right_base_button= self.create_button('move right',
                right_base.create_closure())
        left_right_base_box.addItem(QtGui.QSpacerItem(300,20))
        left_right_base_box.addWidget(left_base_button)
        left_right_base_box.addItem(QtGui.QSpacerItem(50,20))
        left_right_base_box.addWidget(right_base_button)
        left_right_base_box.addItem(QtGui.QSpacerItem(300,20))
        base_box.addLayout(left_right_base_box)

        backward_base_box = QtGui.QHBoxLayout()
        backward_base= Base(Base.BACKWARD, self)
        backward_base_button = self.create_button('move backward',
                backward_base.create_closure())
        backward_base_box.addItem(QtGui.QSpacerItem(400,20))
        backward_base_box.addWidget(backward_base_button)
        backward_base_box.addItem(QtGui.QSpacerItem(400,20))
        base_box.addLayout(backward_base_box)

        large_box.addLayout(base_box)
        
        turn_base_box = QtGui.QHBoxLayout()

        counter_base= Base(Base.COUNTER, self)
        counter_base_button = self.create_button('\\\n        -->',
                counter_base.create_closure())
        
        clockwise_base= Base(Base.CLOCKWISE, self)
        clockwise_base_button = self.create_button('        /\n<--',
                clockwise_base.create_closure())
        turn_base_box.addItem(QtGui.QSpacerItem(75,20))
        turn_base_box.addWidget(counter_base_button)
        turn_base_box.addItem(QtGui.QSpacerItem(225,20))
        turn_base_box.addWidget(clockwise_base_button)
        turn_base_box.addItem(QtGui.QSpacerItem(100,20))
        large_box.addLayout(turn_base_box)

        self._widget.setObjectName('WaterPulse')
        self._widget.setLayout(large_box)
        context.add_widget(self._widget)
        self._widget.setStyleSheet("QWidget { image: url(%s) }" %
                (str(os.path.dirname(os.path.realpath(__file__))) +
                "/../../rosie_background.jpg"))

    def show_text_in_rviz(self, text):
        marker = Marker(type=Marker.TEXT_VIEW_FACING, id=0,
                lifetime=rospy.Duration(1.5),
                pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.06, 0.06, 0.06),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8), text=text)
        self.marker_publisher.publish(marker)
   

 
    def sound_cb(self, sound_request):
        qWarning('Received sound.')
        self.sound_sig.emit(sound_request)
        
    def create_button(self, name, method):
        btn = QtGui.QPushButton(name, self._widget)
        btn.clicked.connect(method)
        btn.setAutoRepeat(True)
        return btn

    def sound_sig_cb(self, sound_request):
        qWarning('Received sound signal.')
        #if (sound_request.command == SoundRequest.SAY):
        qWarning('Robot said: ' + sound_request.arg)

    def command_cb(self):
        button_name = self._widget.sender().text()
        if (button_name == 'Speak'):
            qWarning('Robot will say: ' + self.sound_textbox.text())
            self._sound_client.say(self.sound_textbox.text())
        elif (button_name == 'Trick or Treat'):
            qWarning('Robot will say: Trick or Treat')
            self._sound_client.say('Trick or Treat')
        elif (button_name == 'Thank You'):
            qWarning('Robot will say: Thank You')
            self._sound_client.say('Thank You')

    def knock(self):
        for position in WaterPulse.positions:
            velocities = [0] * len(position)
            traj_goal = JointTrajectoryGoal()
            traj_goal.trajectory.header.stamp = (rospy.Time.now() +
                    rospy.Duration(0.1))
            time_to_joint = 2.0
            traj_goal.trajectory.points.append(JointTrajectoryPoint(
                                  positions=position,
                                  velocities=velocities,
                                  time_from_start=rospy.Duration(time_to_joint)))

            traj_goal.trajectory.joint_names = self.l_joint_names
            self.l_traj_action_client.send_goal(traj_goal)
            result = 0
            while(result < 2): # ACTIVE or PENDING
                self.l_traj_action_client.wait_for_result()
                result = self.l_traj_action_client.get_result()
            #self.l_traj_action_client.wait_for_result()
            #time.sleep(1)
        
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

