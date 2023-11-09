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
from ur_dashboard_msgs.srv import Load, LoadRequest



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
        # if self.hw_version == 'cb3':
        self.quit_proxy = rospy.ServiceProxy('/ur_hardware_interface/dashboard/quit', Trigger)
        self.quit_proxy.wait_for_service()
        self.connect_proxy = rospy.ServiceProxy('/ur_hardware_interface/dashboard/connect', Trigger)
        self.connect_proxy.wait_for_service()

        self.close_safety_popup_proxy = rospy.ServiceProxy('/ur_hardware_interface/dashboard/close_safety_popup', Trigger)
        self.close_safety_popup_proxy.wait_for_service()

        self.unlock_proxy = rospy.ServiceProxy('/ur_hardware_interface/dashboard/brake_release', Trigger)
        self.unlock_proxy.wait_for_service()

        self.load_program_proxy = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
        self.load_program_proxy.wait_for_service()

        self.stop_proxy = rospy.ServiceProxy('/ur_hardware_interface/dashboard/stop', Trigger)
        self.stop_proxy.wait_for_service()

        self.start_proxy = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
        self.start_proxy.wait_for_service()

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
        # TODO: If these services are not available should we consider an automated reboot?
        while True:
            try:
                rospy.sleep(1.0)
                rospy.logwarn('Waiting for UR services')
                self.quit_proxy.wait_for_service(5.0)
            except rospy.exceptions.ROSException as e:
                rospy.logerr("Unable to contact UR services - try a reboot...")
                continue

            try:
                rospy.sleep(1.0)
                rospy.logwarn('Attempting to reset UR connection')
                self.quit_proxy(TriggerRequest())
            except Exception as e:
                rospy.logerr("Unable to contact UR services - try a reboot...")
                continue

            break

        rospy.sleep(1.0)
        self.connect_proxy(TriggerRequest())
        rospy.sleep(1.0)

        # NOTE: Commented out naughty stuff...
        # self.close_safety_popup_proxy(TriggerRequest())
        # rospy.sleep(1.0)
        # self.unlock_proxy(TriggerRequest())

        valid_robot_states = [RobotMode.RUNNING, RobotMode.IDLE]
        while not self.robot_state and self.robot_state.mode not in valid_robot_states:
            rospy.logwarn(f'Waiting for robot to recover state - currently state {self.robot_state} - goal states {valid_robot_states}')
            rospy.sleep(1)

        # --- Stop / Start of the program
        print('Reset')
        self.stop_proxy(TriggerRequest())
        rospy.sleep(1.0)

        # NOTE: Commented out naughty stuff...
        # req = LoadRequest('/programs/ros-control.urp')
        # self.load_program_proxy(req)
        # rospy.sleep(1.0)

        self.start_proxy(TriggerRequest())
        rospy.sleep(1.0)
        # ---
        
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
