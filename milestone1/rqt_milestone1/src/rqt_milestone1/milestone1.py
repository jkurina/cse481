#!/usr/bin/env python

import roslib
roslib.load_manifest('sound_play')
roslib.load_manifest('rospy')
roslib.load_manifest('visualization_msgs')
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, Twist
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
from navigation import move_to_shelf
from gripper import Gripper
from torso import Torso
from marker_perception import ReadMarkers
import os
import time


class Milestone1GUI(Plugin):

    RECEIVE_FROM_HUMAN_R_POS = [0.00952670015493673, 0.3270780665526253,
    0.03185028324084582, -1.3968658009779173, -3.058799671876592,
    -1.1083678332942686, -1.6314425515258866]
    
    READ_FIDUCIAL_R_POS = [0.41004856860653505, 0.29772362823537935, 
    -0.019944325676627628, -1.8394298656773618, -0.44139252862458106,
    -0.09922194050427624, -4.761735317011306]
    
    NAVIGATE_R_POS = [-0.3594077470836499, 0.971353000916152, -1.9647276598906076,
    -1.431900313132731, -1.1839177367219755, -0.09817772642188527, -1.622044624784374]
    
    PLACE_ON_SHELF_R_POS = [-0.15015144487461773, 0.4539704512093072,
    -0.10846018983280459, -0.9819529421527269, -3.0207362886631235, -0.4990689162195487,
    -1.6026396464199553]
    
    RELEASE_BOOK_R_POS = [-0.15545746838546493, 0.44145040258984786,
    -0.13267376861465774, -0.972108533778647, -3.028545645401449, -0.38572817936010495,
    0.008017066746929924]
    
    TUCKED_UNDER_L_POS = [0.05688828299939419, 1.2955758539090194, 1.7180547710154033,
    -1.4511548177467404, -0.28718096455759035, -0.0938208188421713, -10.980395466225648]
    
    sound_sig = Signal(SoundRequest)

    def __init__(self, context):
        super(Milestone1GUI, self).__init__(context)
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
        self.r_traj_action_client = SimpleActionClient(r_traj_controller_name,
                JointTrajectoryAction)
        rospy.loginfo('Waiting for a response from the trajectory action server for RIGHT arm...')
        self.r_traj_action_client.wait_for_server()
        
        l_traj_controller_name = '/l_arm_controller/joint_trajectory_action'
        self.l_traj_action_client = SimpleActionClient(l_traj_controller_name,
                JointTrajectoryAction)
        rospy.loginfo('Waiting for a response from the trajectory action server for LEFT arm...')
        self.l_traj_action_client.wait_for_server()

        #init torso
        self.torso = Torso()

        #init gripper
        self.l_gripper = Gripper('l')
        self.r_gripper = Gripper('r')

        self._widget = QWidget()
        self._widget.setFixedSize(600, 600)
        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
        
        large_box = QtGui.QVBoxLayout()

        #Button for outreaching to recieve book from Human
        button_box = QtGui.QVBoxLayout()
        box_1 = QtGui.QHBoxLayout()
        box_2 = QtGui.QHBoxLayout()
        box_3 = QtGui.QHBoxLayout()
        box_4 = QtGui.QHBoxLayout()
        box_5 = QtGui.QHBoxLayout()

        box_1.addItem(QtGui.QSpacerItem(15,2))
        box_1.addWidget(self.create_button('Prepare To Take', self.command_cb))
        box_1.addItem(QtGui.QSpacerItem(445,2))

        box_2.addItem(QtGui.QSpacerItem(15,2))
        box_2.addWidget(self.create_button('Take From Human', self.command_cb))
        box_2.addItem(QtGui.QSpacerItem(445,2))

        box_3.addItem(QtGui.QSpacerItem(15,2))
        box_3.addWidget(self.create_button('Prepare To Navigate', self.command_cb))
        box_3.addItem(QtGui.QSpacerItem(445,2))

        # Button to move to shelf
        box_5.addItem(QtGui.QSpacerItem(15,2))
        box_5.addWidget(self.create_button('Navigate', move_to_shelf))
        box_5.addItem(QtGui.QSpacerItem(445,2))

        box_4.addItem(QtGui.QSpacerItem(15,2))
        box_4.addWidget(self.create_button('Place On Shelf', self.command_cb))
        box_4.addItem(QtGui.QSpacerItem(445,2))

        button_box.addItem(QtGui.QSpacerItem(20,120))
        button_box.addLayout(box_1)
        button_box.addLayout(box_2)
        button_box.addLayout(box_3)
        button_box.addLayout(box_5)
        button_box.addLayout(box_4)
        button_box.addItem(QtGui.QSpacerItem(20,240))
        large_box.addLayout(button_box)
        self.marker_perception = ReadMarkers()
        self._widget.setObjectName('Milestone1GUI')
        self._widget.setLayout(large_box)
        context.add_widget(self._widget)
        self._widget.setStyleSheet("QWidget { image: url(%s) }" %
                (str(os.path.dirname(os.path.realpath(__file__))) +
                "/../../librarian_gui_background.jpg"))

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
        if (button_name == 'Prepare To Take'):
            # Open gripper and move arms to take book
            self.torso.move(.1) # move torso .1 meter from base (MAX is .2)
            self.saved_l_arm_pose = Milestone1GUI.TUCKED_UNDER_L_POS
            self.saved_r_arm_pose = Milestone1GUI.RECEIVE_FROM_HUMAN_R_POS       
            self.move_arm('l', 5.0)
            self.move_arm('r', 5.0)  # Increase these numbers for slower movement
            self.l_gripper.close_gripper()
            self.r_gripper.open_gripper(True)
            self._sound_client.say("Please give me a book")
        elif (button_name == 'Take From Human'):
            # Close gripper and move arms to see book
            self.r_gripper.close_gripper()
            self._sound_client.say("Thank you")
            time.sleep(3)
            self.saved_r_arm_pose = Milestone1GUI.READ_FIDUCIAL_R_POS
            self.move_arm('r', 5.0)  # Increase these numbers for slower movement
            rospy.loginfo("id is: " + str(self.marker_perception.get_marker_id()))
        elif (button_name == 'Prepare To Navigate'):
            # Tuck arms
            self.saved_r_arm_pose = Milestone1GUI.NAVIGATE_R_POS
            self.move_arm('r', 5.0)  # Increase these numbers for slower movement
        elif (button_name == 'Place On Shelf'):
            # Move forward, place book on the shelf, and move back
            self.saved_r_arm_pose = Milestone1GUI.PLACE_ON_SHELF_R_POS
            self.move_arm('r', 5.0, True)  # Increase these numbers for slower movement
            self.move_base(True)
            self.r_gripper.open_gripper(True)
            self.saved_r_arm_pose = Milestone1GUI.RELEASE_BOOK_R_POS
            self.move_arm('r', 2.0, True)  # Increase these numbers for slower movement
            self.move_base(False)
  
    # Moves forward to the bookshelf (or backward if isForward is false)
    def move_base(self, isForward):
        topic_name = '/base_controller/command'
        base_publisher = rospy.Publisher(topic_name, Twist)

        distance = 7.0
        if not isForward:
            distance *= -1

        twist_msg = Twist()
        twist_msg.linear = Vector3(distance, 0.0, 0.0)
        twist_msg.angular = Vector3(0.0, 0.0, 0.0)
        for x in range(0, 10):
            rospy.loginfo("Moving the base")
            base_publisher.publish(twist_msg)
            time.sleep(0.1)

    # Moves arms using kinematics
    def move_arm(self, side_prefix, time_to_joints = 2.0, wait = False):
        # forward kinematics
        if (side_prefix == 'r'):
            if self.saved_r_arm_pose is None:
                rospy.logerr('Target pose for right arm is None, cannot move.')
            else:
                self.freeze_arm(side_prefix)
                self.move_to_joints(side_prefix, self.saved_r_arm_pose, time_to_joints, wait)
        else: # side_prefix == 'l'
            if self.saved_l_arm_pose is None:
                rospy.logerr('Target pose for left arm is None, cannot move.')
            else:
                self.freeze_arm(side_prefix)
                self.move_to_joints(side_prefix, self.saved_l_arm_pose, time_to_joints, wait)
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

    def move_to_joints(self, side_prefix, positions, time_to_joint, wait = False):
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
        
        if side_prefix == 'r':
            traj_goal.trajectory.joint_names = self.r_joint_names 
            action_client = self.r_traj_action_client
        else:
            traj_goal.trajectory.joint_names = self.l_joint_names 
            action_client = self.l_traj_action_client
        action_client.send_goal(traj_goal)
        if wait:
            result = 0
            while(result < 2): # ACTIVE or PENDING
                action_client.wait_for_result()
                result = action_client.get_result()

