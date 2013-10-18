#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('trajectory_msgs')
roslib.load_manifest('control_msgs')
roslib.load_manifest('pr2_mechanism_msgs')
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('actionlib')

from subprocess import call
import threading
import rospy
from qt_gui.plugin import Plugin
from python_qt_binding import QtGui,QtCore
from python_qt_binding.QtGui import QWidget, QFrame
from python_qt_binding.QtGui import QGroupBox
from python_qt_binding.QtCore import QSignalMapper, qWarning, Signal
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryGoal
from control_msgs.msg import JointTrajectoryAction
from pr2_mechanism_msgs.srv import SwitchController
from sensor_msgs.msg import JointState
from actionlib import SimpleActionClient

from pose_saver import PoseSaver
from pose_loader import PoseLoader

class ArmGUI(Plugin):

    joint_sig = Signal(JointState)

    def __init__(self, context):
        super(ArmGUI, self).__init__(context)
        self.setObjectName('ArmGUI')
        self._widget = QWidget()
        
        # Action/service/message clients or servers
        
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

        self.all_joint_names = []
        self.all_joint_poses = []

        self.saved_r_arm_pose = None
        self.saved_l_arm_pose = None

        self.lock = threading.Lock()
        rospy.Subscriber('joint_states', JointState, self.joint_states_cb)


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
        self.joint_sig.connect(self.joint_sig_cb)
        
        large_box = QtGui.QVBoxLayout()
        
        button_box1 = QtGui.QHBoxLayout()
        button_box1.addWidget(self.create_button('Relax right arm'))
        button_box1.addWidget(self.create_button('Freeze right arm'))
        button_box1.addWidget(self.create_button('Relax left arm'))
        button_box1.addWidget(self.create_button('Freeze left arm'))
        button_box1.addStretch(1)
        large_box.addLayout(button_box1)
        large_box.addItem(QtGui.QSpacerItem(100,20))

        button_box2 = QtGui.QHBoxLayout()
        button_box2.addWidget(self.create_button('Save right arm pose'))
        button_box2.addWidget(self.create_button('Save left arm pose'))
        button_box2.addWidget(self.create_button('Move right arm to saved pose'))
        button_box2.addWidget(self.create_button('Move left arm to saved pose'))
        button_box2.addStretch(1)
        large_box.addLayout(button_box2)
        large_box.addItem(QtGui.QSpacerItem(100,20))

        state_box = QtGui.QHBoxLayout()
        self.r_state_label = QtGui.QLabel('Saved right arm pose: None')
        self.l_state_label = QtGui.QLabel('Saved left arm pose: None')
        state_box.addWidget(self.r_state_label)
        state_box.addItem(QtGui.QSpacerItem(50,20))
        state_box.addWidget(self.l_state_label)
        state_box.addStretch(1)
        large_box.addLayout(state_box)

        left_pose_saver = PoseSaver(PoseSaver.LEFT, self)
        right_pose_saver = PoseSaver(PoseSaver.RIGHT, self)
        large_box.addWidget(self.create_button("Create left arm pose",
              left_pose_saver.create_closure()))
        large_box.addWidget(self.create_button("Create right arm pose",
              right_pose_saver.create_closure()))

        left_pose_loader = PoseLoader(PoseLoader.LEFT, self)
        right_pose_loader = PoseLoader(PoseLoader.RIGHT, self)
        large_box.addWidget(left_pose_loader.create_button())
        large_box.addWidget(right_pose_loader.create_button())

        large_box.addStretch(1)
        self._widget.setObjectName('ArmGUI')
        self._widget.setLayout(large_box)
        context.add_widget(self._widget)
        rospy.loginfo('GUI initialization complete.')
        
    def create_button(self, name, method=None):
        if method == None:
          method = self.command_cb
        btn = QtGui.QPushButton(name, self._widget)
        btn.clicked.connect(method)
        return btn

    def command_cb(self):
        button_name = self._widget.sender().text()
        if (button_name == 'Relax right arm'):
            self.relax_arm('r')
        elif (button_name == 'Freeze right arm'):
            self.freeze_arm('r')
        elif (button_name == 'Relax left arm'):
            self.relax_arm('l')
        elif (button_name == 'Freeze left arm'):
            self.freeze_arm('l')
        elif (button_name == 'Save right arm pose'):
            self.save_pose('r')
        elif (button_name == 'Save left arm pose'):
            self.save_pose('l')
        elif (button_name == 'Move right arm to saved pose'):
            self.move_arm('r')
        elif (button_name == 'Move left arm to saved pose'):
            self.move_arm('l')

    def save_pose(self, side_prefix):
        if (side_prefix == 'r'):
            self.saved_r_arm_pose = self.get_joint_state('r')
            self.r_state_label.setText('Saved right arm pose: ' + str(self.saved_r_arm_pose))
        else:
            self.saved_l_arm_pose = self.get_joint_state('l')
            self.l_state_label.setText('Saved left arm pose: ' + str(self.saved_l_arm_pose))

    def move_arm(self, side_prefix):
        if (side_prefix == 'r'):
            if self.saved_r_arm_pose is None:
                rospy.logerr('Target pose for right arm is None, cannot move.')
            else:
                self.freeze_arm('r')
                self.move_to_joints('r', self.saved_r_arm_pose, 2.0)
        else:
            if self.saved_l_arm_pose is None:
                rospy.logerr('Target pose for left arm is None, cannot move.')
            else:
                self.freeze_arm('l')
                self.move_to_joints('l', self.saved_l_arm_pose, 2.0)
                pass


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

    def relax_arm(self, side_prefix):
        controller_name = side_prefix + '_arm_controller'
        start_controllers = []
        stop_controllers = [controller_name]
        self.set_arm_mode(start_controllers, stop_controllers)

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


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        # Leave both arm controllers on
        start_controllers = ['r_arm_controller', 'l_arm_controller']
        stop_controllers = []
        self.set_arm_mode(start_controllers, stop_controllers)

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

