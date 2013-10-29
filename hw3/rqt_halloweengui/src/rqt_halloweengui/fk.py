''' Interface for controlling one arm '''
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('kinematics_msgs')
roslib.load_manifest('sensor_msgs')

import threading
import rospy
from numpy import array, sign, pi
from arm_navigation_msgs.msg import RobotState, MultiDOFJointState
from kinematics_msgs.srv import GetKinematicSolverInfo
from kinematics_msgs.srv import GetPositionFK, GetPositionFKRequest
from sensor_msgs.msg import JointState
 


class FK:

    def __init__(self, side_prefix, gui):
        self.side_prefix = side_prefix
        self.gui = gui

        # Set up Inversse Kinematics services
        fk_info_srv_name = ('pr2_' + self._side() + '_arm_kinematics/get_fk_solver_info')
        fk_srv_name = 'pr2_' + self._side() + '_arm_kinematics/get_fk'

        rospy.loginfo('Waiting for FK info service to respond.')
        rospy.wait_for_service(fk_info_srv_name)
        fk_info_srv = rospy.ServiceProxy(fk_info_srv_name, GetKinematicSolverInfo)
        solver_info = fk_info_srv()
        self.fk_joints = solver_info.kinematic_solver_info.joint_names
        self.fk_limits = solver_info.kinematic_solver_info.limits
        print(solver_info)

        rospy.loginfo('Waiting for FK service to respond.')
        rospy.wait_for_service(fk_srv_name)
        self.fk_srv = rospy.ServiceProxy(fk_srv_name,
                GetPositionFK,
                persistent=True)

        # Set up common parts of an FK request
        self.fk_request = GetPositionFKRequest()
#        self.fk_request.timeout = rospy.Duration(4.0)
        self.fk_request.fk_link_names = solver_info.kinematic_solver_info.link_names
#        self.fk_request.fk_request.header.frame_id = 'base_link'
#        self.fk_request.fk_request.fk_seed_state.joint_state.name = self.fk_joints
#        self.fk_request.fk_request.fk_seed_state.joint_state.position = [0] * len(self.fk_joints)

    def _side(self):
        if (self.side_prefix == 'r'):
            return 'right'
        else:
            return 'left'

    def get_ee_for_state(self, joint_state, multi_dof_joint_state):
        self.fk_request.robot_state = RobotState()
        self.fk_request.robot_state.joint_state = joint_state
        self.fk_request.robot_state.multi_dof_joint_state = multi_dof_joint_state
        try:
            rospy.loginfo('Requesting FK.')
            response = self.fk_srv(self.fk_request)
            if(response.error_code.val == response.error_code.SUCCESS):
                pose = response.pose_stamped.pose
            else:
                rospy.logwarn('Could not find FK solution.')
                return None
        except rospy.ServiceException:
            rospy.logerr('Exception while getting the FK solution.')
            return None

        return pose

    def get_ee_for_joints(self, joints):
        if self.side_prefix == 'l':
            joint_names = self.gui.l_joint_names
        else: 
            joint_names = self.gui.r_joint_names
        joint_state = JointState()
        joint_state.header = self.fk_request.header
        joint_state.name = joint_names
        joint_state.position = joints
            
        multi_dof_joint_state = MultiDOFJointState()
        # TODO: build it
        return self.get_ee_for_state(joint_state, multi_dof_joint_state)
