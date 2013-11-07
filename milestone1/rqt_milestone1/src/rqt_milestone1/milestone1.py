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
from pr2_mechanism_msgs.srv import SwitchController
from control_msgs.msg import JointTrajectoryGoal
from control_msgs.msg import JointTrajectoryAction
from sensor_msgs.msg import JointState
from actionlib import SimpleActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
import os


class Milestone1GUI(Plugin):

    RECIEVE_FROM_HUMAN_L_POS = [-0.11953699873627144, -0.015476264363818703, 0.05020422194221652, -0.14995566383159287, 50.24253872219175, -0.08894781979101685, 14.235824877202386]
    RECIEVE_FROM_HUMAN_R_POS = [0.09367691677227741, -0.01756165017492318, -0.08136022417629407, -0.1544435558844821, 6.30432982194113, -0.09030261188385946, -32.993813431756145]
    READ_FIDUCIAL_L_POS = [0.1110873064172444, -0.09318951143030572, -0.10445131917355743, -1.400340298051122, 6.288248331768874, -0.23932066322496848, -33.10010572355945] 
    READ_FIDUCIAL_R_POS = [-0.09035386942661228, -0.09296521392749924, 0.11739289419119903, -1.4097503942910512, 50.25757896479891, -0.12453811643248647, 14.217551130760555]
    PLACE_ON_SHELF_L_POS = [0.09516923588470316, 0.029726911840466965, -0.15544415395918154, -0.14879749814052468, 6.490828830269738, -0.08351522034832204, -33.09301376958322]
    PLACE_ON_SHELF_R_POS = [-0.08952480325304246, 0.03155851288225806, 0.4005795175604179, -0.14908703956329128, 49.939015056962155, -0.08494499914185327, 14.156203553420127]
    sound_sig = Signal(SoundRequest)

    def __init__(self, context):
        super(Milestone1GUI, self).__init__(context)
        self._widget = QWidget()
        self._widget.setFixedSize(600, 600)
        self._sound_client = SoundClient()
        rospy.Subscriber('robotsound', SoundRequest, self.sound_cb)
        switch_srv_name = 'pr2_controller_manager/switch_controller'
        rospy.loginfo('Waiting for switch controller service...')
        rospy.wait_for_service(switch_srv_name)
        self.switch_service_client = rospy.ServiceProxy(switch_srv_name,
                                                 SwitchController)

        self.r_joint_names = ['r_shoulder_pan_joint',
                              'r_shoulder_lift_joint',
                              'r_upper_arm_roll_joint',
                              'r_elbow_flex_joint',
                              'r_forearm_roll_joint',
                              'r_wrist_flex_joint',
                              'r_wrist_roll_joint']
        self.l_joint_names = ['l_shoulder_pan_joint',
                              'l_shoulder_lift_joint',
                              'l_upper_arm_roll_joint',
                              'l_elbow_flex_joint',
                              'l_forearm_roll_joint',
                              'l_wrist_flex_joint',
                              'l_wrist_roll_joint']

        # Create a trajectory action client
        r_traj_controller_name = '/r_arm_controller/joint_trajectory_action'
        self.r_traj_action_client = SimpleActionClient(r_traj_controller_name, JointTrajectoryAction)
        rospy.loginfo('Waiting for a response from the trajectory action server for RIGHT arm...')
        self.r_traj_action_client.wait_for_server()
        
        l_traj_controller_name = '/l_arm_controller/joint_trajectory_action'
        self.l_traj_action_client = SimpleActionClient(l_traj_controller_name, JointTrajectoryAction)
        rospy.loginfo('Waiting for a response from the trajectory action server for LEFT arm...')
        self.l_traj_action_client.wait_for_server()

        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
        
        large_box = QtGui.QVBoxLayout()

        #Button for outreaching to recieve book from Human
        button_box = QtGui.QHBoxLayout()
        button_box.addItem(QtGui.QSpacerItem(15,20))
        button_box.addWidget(self.create_button('Prepare To Take', self.command_cb))
        button_box.addWidget(self.create_button('Take From Human', self.command_cb))
        button_box.addWidget(self.create_button('Place On Shelf', self.command_cb))
        button_box.addStretch(1)
        large_box.addLayout(button_box)


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
        if (button_name == 'Prepare to Take'):
            self.saved_l_arm_pose = Milestone1GUI.RECIEVE_FROM_HUMAN_L_POS
            self.saved_r_arm_pose = Milestone1GUI.RECIEVE_FROM_HUMAN_R_POS
            self.move_arm('l')
            self.move_arm('r')
        elif (button_name == 'Take From Human'):
            self.saved_l_arm_pose = Milestone1GUI.READ_FIDUCIAL_L_POS
            self.saved_r_arm_pose = Milestone1GUI.READ_FIDUCIAL_R_POS
            self.move_arm('l')
            self.move_arm('r')
            #CALL FIDUCIAL RECOGNITION
        elif (button_name == 'Place On Shelf'): 
            self.saved_l_arm_pose = Milestone1GUI.PLACE_ON_SHELF_L_POS
            self.saved_r_arm_pose = Milestone1GUI.PLACE_ON_SHELF_R_POS
            self.move_arm('l')
            self.move_arm('r')
    
    def move_arm(self, side_prefix):
        # forward kinematics
        if (side_prefix == 'r'):
            if self.saved_r_arm_pose is None:
                rospy.logerr('Target pose for right arm is None, cannot move.')
            else:
                self.freeze_arm(side_prefix)
                time_to_joints = 2.0 
                self.move_to_joints(side_prefix, self.saved_r_arm_pose, time_to_joints)
        else: # side_prefix == 'l'
            if self.saved_l_arm_pose is None:
                rospy.logerr('Target pose for left arm is None, cannot move.')
            else:
                self.freeze_arm(side_prefix)
                time_to_joints = 2.0
                self.move_to_joints(side_prefix, self.saved_l_arm_pose, time_to_joints)
                pass

    def freeze_arm(self, side_prefix):
        controller_name = side_prefix + '_arm_controller'
        start_controllers = [controller_name]
        stop_controllers = []
        self.set_arm_mode(start_controllers, stop_controllers)

    def set_arm_mode(self, start_controllers, stop_controllers):
        try:
            self.switch_service_client(start_controllers, stop_controllers, 1)
        except rospy.ServiceException:
            rospy.logerr('Could not change arm mode.')

    def joint_states_cb(self, msg):
        '''Callback function that saves the joint positions when a
            joint_states message is received'''
        self.lock.acquire()
        self.all_joint_names = msg.name
        self.all_joint_poses = msg.position
        self.joint_sig.emit(msg)
        self.lock.release()

    def joint_sig_cb(self, msg):
        pass

    def get_joint_state(self, side_prefix):
        '''Returns position for arm joints on the requested side (r/l)'''
        if side_prefix == 'r':
            joint_names = self.r_joint_names
        else:
            joint_names = self.l_joint_names

        if self.all_joint_names == []:
            rospy.logerr("No robot_state messages received yet!\n")
            return None
    
        positions = []
        self.lock.acquire()
        for joint_name in joint_names:
            if joint_name in self.all_joint_names:
                index = self.all_joint_names.index(joint_name)
                position = self.all_joint_poses[index]
                positions.append(position)
            else:
                rospy.logerr("Joint %s not found!", joint_name)
                self.lock.release()
                return None

        self.lock.release()
        return positions

    def move_to_joints(self, side_prefix, positions, time_to_joint):
        '''Moves the arm to the desired joints'''
        velocities = [0] * len(positions)
        traj_goal = JointTrajectoryGoal()
        traj_goal.trajectory.header.stamp = (rospy.Time.now() + rospy.Duration(0.1))
        traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=positions,
                            velocities=velocities, time_from_start=rospy.Duration(time_to_joint)))
        
        if (side_prefix == 'r'):
            traj_goal.trajectory.joint_names = self.r_joint_names
            self.r_traj_action_client.send_goal(traj_goal)
        else:
            traj_goal.trajectory.joint_names = self.l_joint_names
            self.l_traj_action_client.send_goal(traj_goal)

