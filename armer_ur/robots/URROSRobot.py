"""
URROSRobot module defines the URROSRobot type
URROSRobot provides robot-specific callbacks for recovery and setting impedance.
.. codeauthor:: Gavin Suddreys
"""
import rospy
import actionlib
import roboticstoolbox as rtb

from armer.robots import ROSRobot
from rospy.core import rospyinfo

from std_srvs.srv import EmptyRequest, EmptyResponse
from std_srvs.srv import Trigger, TriggerRequest

from armer_msgs.msg import ManipulatorState

from armer_msgs.srv import \
    SetCartesianImpedanceRequest, \
    SetCartesianImpedanceResponse

from ur_dashboard_msgs.msg import RobotMode, SafetyMode



class URROSRobot(ROSRobot):
    def __init__(self,
                 robot: rtb.robot.Robot,
                 hw_version: str = None,
                 controller_name: str = None,
                 recover_on_estop: bool = True,
                 *args,
                 **kwargs):
        super().__init__(robot, *args, **kwargs)
        
        # ----- Debugging Output ----------------
        rospy.loginfo(f"Armer UR -> Configured hw_version: {hw_version}")

        self.hw_version = hw_version \
            if hw_version else 'cb3'

        self.controller_name = controller_name \
            if controller_name else self.joint_velocity_topic.split('/')[1]
        
        rospy.loginfo(f"controller name: {self.controller_name}")

        self.recover_on_estop = recover_on_estop
        self.last_estop_state = 0

        # UR state subscribers
        self.robot_state_subscriber = rospy.Subscriber(
            '/ur_hardware_interface/robot_mode',
            RobotMode,
            self.ur_robot_cb
        )
        self.safety_state_subscriber = rospy.Subscriber(
            '/ur_hardware_interface/safety_mode',
            SafetyMode,
            self.ur_safety_cb
        )
        self.last_estop_state = 0

        self.robot_state = None
        self.safety_state = None

        # Error recovery services
        # NOTE: this section should not be run when using CB2 models
        if self.hw_version == 'cb3':
            self.unlock_proxy = rospy.ServiceProxy('/ur_hardware_interface/dashboard/brake_release', Trigger)
            self.unlock_proxy.wait_for_service()

            self.reset_proxy = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
            self.reset_proxy.wait_for_service()

            self.recover_cb(EmptyRequest())
        

    def recover_cb(self, req: EmptyRequest) -> EmptyResponse: # pylint: disable=no-self-use
        """[summary]
        ROS Service callback:
        Invoke any available error recovery functions on the robot when an error occurs
        :param req: an empty request
        :type req: EmptyRequest
        :return: an empty response
        :rtype: EmptyResponse
        """
        print('Recover')
        self.unlock_proxy(TriggerRequest())
        while not self.robot_state or self.robot_state.mode != RobotMode.RUNNING:
            rospy.sleep(1)
        print('Reset')
        self.reset_proxy(TriggerRequest())
        return EmptyResponse()

    def get_state(self):
        state = super().get_state()

        if self.robot_state:
            state.errors |= ManipulatorState.LOCKED if self.robot_state.mode == RobotMode.IDLE or self.robot_state.mode == RobotMode.POWER_OFF else 0
            
        if self.safety_state:
            state.errors |= ManipulatorState.ESTOP if self.safety_state.mode == SafetyMode.ROBOT_EMERGENCY_STOP else 0
            state.errors |= ManipulatorState.JOINT_LIMIT_VIOLATION | ManipulatorState.CARTESIAN_LIMIT_VIOLATION | ManipulatorState.TORQUE_LIMIT_VIOLATION if self.safety_state.mode == SafetyMode.VIOLATION else 0
            state.errors |= ManipulatorState.OTHER if self.safety_state.mode != SafetyMode.NORMAL and self.safety_state.mode != SafetyMode.ROBOT_EMERGENCY_STOP else 0 

        if self.safety_state and self.safety_state.mode == SafetyMode.NORMAL:
            if self.recover_on_estop and self.last_estop_state == 1:
                self.recover_cb(EmptyRequest())
        else:
            if state.errors & ManipulatorState.OTHER == ManipulatorState.OTHER:
                self.recover_cb(EmptyRequest())

        self.last_estop_state = 1 if self.safety_state and \
            self.safety_state.mode == SafetyMode.ROBOT_EMERGENCY_STOP else 0

        return state

    def ur_robot_cb(self, msg):
        self.robot_state = msg

    def ur_safety_cb(self, msg):
        self.safety_state = msg