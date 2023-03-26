#!/usr/bin/env python3
# Python imports
import rospy
import os
import ruamel.yaml

# Behaviour Tree Packages
from ros_trees.trees import BehaviourTree
from ros_trees.leaves import Leaf
from ros_trees.leaves_common.console import Print
from py_trees.composites import Sequence
from py_trees.decorators import OneShot

# ROS messages
from geometry_msgs.msg import Pose
# Armer Trees Specific Leaves/Branches
from armer_trees.motion import MoveToHomePose, MoveJointsToPose

# Update Path Variable to main package
__path__ = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))

class Wait(Leaf):
    """
    General Wait Leaf (Waits in Seconds)
    """
    def __init__(self, name='Wait', duration=1):
        super().__init__(
            name=name,
            load_fn=self.load_fn,
            eval_fn=self.eval_fn,
            save=False
        )
        self.duration = duration

    def load_fn(self):
        self.start = rospy.get_time()
        return None

    def eval_fn(self, value):
        return True

    def _is_leaf_done(self):
        return rospy.get_time() - self.start > self.duration
    
class JointHandler(Leaf):
    """
    This Leaf Gets the Current Joint Poses of the robot
    """
    def __init__(self,
        name="Joint Handler Leaf",
        save=False,
        identifier=None,
        set=False,
        *args, **kwargs):
        super().__init__(name=name,
            save=save,
            result_fn=self.result_fn,
            *args, **kwargs)
        self.identifier = identifier

    def result_fn(self):
        path = rospy.get_param('~cfg', os.path.join(__path__, 'cfg/demo_poses.yaml'))
        config, ind, bsi = ruamel.yaml.util.load_yaml_guess_indent(open(path))

        joint_poses_cfg = config['joint_poses'] if 'joint_poses' in config else None

        if joint_poses_cfg == None or self.identifier == None:
            rospy.logerr(f"[{self.name}] NO SAVED JOINTS DEFINED OR INPUT ERROR. See /cfg/demo_poses.yaml")
            return None

        # Update the state in the config    
        key = self.identifier
        poses = joint_poses_cfg[0][key]
        rospy.loginfo(f"read poses for {key}: {poses}")
        return poses
    
class ConductMotion(Sequence):
    def __init__(self, 
        name="Sequence",
        identifier=None,
        speed=0.1,
        *args, **kwargs):
        super().__init__(name, [
            # Plan: 
            #   - Get current joint position (base)
            #   - Load in configured joint positions (other joints) and update with current base
            # TODO: get handoff mode and use that (i.e., read app config and get mode as key)
            JointHandler(identifier=identifier, save=True, save_key='updated_joint_pose'),
            Print(load_key='updated_joint_pose'),
            MoveJointsToPose(
                load_key='updated_joint_pose',
                speed=speed
            ),
        ])

if __name__ == '__main__':
    # Create ROS node 
    rospy.init_node('behaviour_pose_demo')

    # ----------- Initialise The Main Tree --------------------------------------------
    demo_tree = BehaviourTree('Demo Agent', Sequence(name="Main Process", children=[
        # Moves to home on startup of application
        OneShot(Sequence(children=[
            Print(load_value="Initialising to Home"),
            MoveToHomePose(speed=0.1)
        ])),
        # Move to first pose
        ConductMotion(identifier='pose_1', speed=0.1),
        # Arbitrary Wait
        Wait(duration=2),
        # Move to second pose
        ConductMotion(identifier='pose_2', speed=0.1),
        # Arbitrary Wait
        Wait(duration=2),
        # Move to third pose
        ConductMotion(identifier='pose_3', speed=0.1),
        # Arbitrary Wait
        Wait(duration=2),
        # Move back to Home
        MoveToHomePose(
            speed=0.1
        )
    ]))
    
    # Run the selected Tree
    demo_tree.run(
        hz=30, 
        push_to_start=False, 
        setup_timeout=5, 
        log_level='INFO'
    )

    # demo_tree.visualise()