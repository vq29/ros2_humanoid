#!/usr/bin/env python3
"""
Pick-and-Place State Machine Node
Coordinates the full pick-and-place pipeline:
  IDLE → DETECT → APPROACH → GRASP → LIFT → MOVE → PLACE → OPEN → HOME → IDLE

Uses MoveIt2 for motion planning and the detected object pose from perception.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from control_msgs.action import GripperCommand
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    RobotState,
    WorkspaceParameters,
)
from shape_msgs.msg import SolidPrimitive

from enum import Enum, auto
import math


class State(Enum):
    IDLE = auto()
    DETECT = auto()
    APPROACH = auto()
    GRASP = auto()
    LIFT = auto()
    MOVE = auto()
    PLACE = auto()
    OPEN_GRIPPER = auto()
    HOME = auto()


class PickPlaceNode(Node):
    """State machine for autonomous pick-and-place."""

    def __init__(self):
        super().__init__('pick_place_node')
        self.callback_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('approach_height', 0.10)   # Height above object for approach
        self.declare_parameter('grasp_height', 0.02)      # Height above table for grasp
        self.declare_parameter('lift_height', 0.15)        # Height to lift after grasp
        self.declare_parameter('place_x', 0.45)            # Target place position
        self.declare_parameter('place_y', 0.15)
        self.declare_parameter('place_z', 0.85)
        self.declare_parameter('gripper_open', 0.04)
        self.declare_parameter('gripper_close', 0.01)

        self.approach_height = self.get_parameter('approach_height').value
        self.grasp_height = self.get_parameter('grasp_height').value
        self.lift_height = self.get_parameter('lift_height').value
        self.place_position = Point(
            x=self.get_parameter('place_x').value,
            y=self.get_parameter('place_y').value,
            z=self.get_parameter('place_z').value,
        )
        self.gripper_open_pos = self.get_parameter('gripper_open').value
        self.gripper_close_pos = self.get_parameter('gripper_close').value

        # State
        self.state = State.IDLE
        self.detected_pose = None

        # Action Clients
        self.move_action_client = ActionClient(
            self, MoveGroup, 'move_action',
            callback_group=self.callback_group
        )
        self.gripper_action_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd',
            callback_group=self.callback_group
        )

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/detected_object_pose',
            self.detected_pose_callback, 10,
            callback_group=self.callback_group
        )

        # State machine timer (2 Hz)
        self.timer = self.create_timer(0.5, self.state_machine_tick)

        # Named joint positions
        self.home_joints = {
            'shoulder_pitch_joint': 0.0,
            'shoulder_roll_joint': 0.0,
            'shoulder_yaw_joint': 0.0,
            'elbow_pitch_joint': 0.0,
            'wrist_pitch_joint': 0.0,
            'wrist_roll_joint': 0.0,
        }
        self.ready_joints = {
            'shoulder_pitch_joint': -0.5,
            'shoulder_roll_joint': 0.0,
            'shoulder_yaw_joint': 0.0,
            'elbow_pitch_joint': 1.0,
            'wrist_pitch_joint': -0.5,
            'wrist_roll_joint': 0.0,
        }

        self.busy = False  # Prevent concurrent actions

        self.get_logger().info('Pick-and-place node initialized. Waiting for action servers...')

    def detected_pose_callback(self, msg: PoseStamped):
        """Store the latest detected object pose."""
        self.detected_pose = msg

    def state_machine_tick(self):
        """Main state machine loop."""
        if self.busy:
            return

        if self.state == State.IDLE:
            self.get_logger().info('State: IDLE → Starting pick-and-place sequence')
            self.state = State.DETECT

        elif self.state == State.DETECT:
            if self.detected_pose is not None:
                self.get_logger().info(
                    f'State: DETECT → Object found at '
                    f'({self.detected_pose.pose.position.x:.3f}, '
                    f'{self.detected_pose.pose.position.y:.3f}, '
                    f'{self.detected_pose.pose.position.z:.3f})'
                )
                # Open gripper first
                self.busy = True
                self.send_gripper_command(self.gripper_open_pos, State.APPROACH)
            else:
                self.get_logger().info('State: DETECT → Waiting for object detection...',
                                       throttle_duration_sec=5.0)

        elif self.state == State.APPROACH:
            self.get_logger().info('State: APPROACH → Moving above object')
            self.busy = True
            # Move to approach pose (above the object)
            approach_pose = Pose()
            approach_pose.position.x = self.detected_pose.pose.position.x
            approach_pose.position.y = self.detected_pose.pose.position.y
            approach_pose.position.z = self.detected_pose.pose.position.z + self.approach_height
            # Gripper pointing down
            approach_pose.orientation.x = 0.0
            approach_pose.orientation.y = 1.0
            approach_pose.orientation.z = 0.0
            approach_pose.orientation.w = 0.0
            self.send_move_command(approach_pose, State.GRASP)

        elif self.state == State.GRASP:
            self.get_logger().info('State: GRASP → Moving down to grasp')
            self.busy = True
            # Move down to grasp height
            grasp_pose = Pose()
            grasp_pose.position.x = self.detected_pose.pose.position.x
            grasp_pose.position.y = self.detected_pose.pose.position.y
            grasp_pose.position.z = self.detected_pose.pose.position.z + self.grasp_height
            grasp_pose.orientation.x = 0.0
            grasp_pose.orientation.y = 1.0
            grasp_pose.orientation.z = 0.0
            grasp_pose.orientation.w = 0.0
            self.send_move_command(grasp_pose, State.LIFT)

        elif self.state == State.LIFT:
            self.get_logger().info('State: LIFT → Closing gripper and lifting')
            self.busy = True
            # Close gripper, then lift
            self.send_gripper_command(self.gripper_close_pos, State.MOVE)

        elif self.state == State.MOVE:
            self.get_logger().info('State: MOVE → Moving to place location')
            self.busy = True
            # Move to place position
            place_pose = Pose()
            place_pose.position = self.place_position
            place_pose.orientation.x = 0.0
            place_pose.orientation.y = 1.0
            place_pose.orientation.z = 0.0
            place_pose.orientation.w = 0.0
            self.send_move_command(place_pose, State.PLACE)

        elif self.state == State.PLACE:
            self.get_logger().info('State: PLACE → Lowering to place')
            self.busy = True
            place_lower = Pose()
            place_lower.position.x = self.place_position.x
            place_lower.position.y = self.place_position.y
            place_lower.position.z = self.place_position.z - self.lift_height + self.grasp_height
            place_lower.orientation.x = 0.0
            place_lower.orientation.y = 1.0
            place_lower.orientation.z = 0.0
            place_lower.orientation.w = 0.0
            self.send_move_command(place_lower, State.OPEN_GRIPPER)

        elif self.state == State.OPEN_GRIPPER:
            self.get_logger().info('State: OPEN_GRIPPER → Releasing object')
            self.busy = True
            self.send_gripper_command(self.gripper_open_pos, State.HOME)

        elif self.state == State.HOME:
            self.get_logger().info('State: HOME → Returning to home position')
            self.busy = True
            self.send_joint_command(self.home_joints, State.IDLE)

    def send_gripper_command(self, position, next_state):
        """Send a gripper command and transition to next state on completion."""
        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Gripper action server not available')
            self.busy = False
            return

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 20.0

        future = self.gripper_action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._action_response_callback(f, next_state, 'gripper')
        )

    def send_move_command(self, target_pose, next_state):
        """Send a MoveGroup goal to move the arm to target_pose."""
        if not self.move_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available')
            self.busy = False
            return

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = 'arm'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5

        # Workspace bounds
        goal.request.workspace_parameters = WorkspaceParameters()
        goal.request.workspace_parameters.header.frame_id = 'world'
        goal.request.workspace_parameters.min_corner.x = -1.0
        goal.request.workspace_parameters.min_corner.y = -1.0
        goal.request.workspace_parameters.min_corner.z = -0.1
        goal.request.workspace_parameters.max_corner.x = 1.5
        goal.request.workspace_parameters.max_corner.y = 1.0
        goal.request.workspace_parameters.max_corner.z = 2.0

        # Position constraint
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'world'
        position_constraint.link_name = 'grasp_frame'
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0

        bounding_volume = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]  # Tolerance sphere radius
        bounding_volume.primitives.append(primitive)

        primitive_pose = Pose()
        primitive_pose.position = target_pose.position
        primitive_pose.orientation.w = 1.0
        bounding_volume.primitive_poses.append(primitive_pose)

        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0

        # Orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = 'world'
        orientation_constraint.link_name = 'grasp_frame'
        orientation_constraint.orientation = target_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0

        # Combine constraints
        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        goal.request.goal_constraints.append(constraints)

        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        future = self.move_action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._action_response_callback(f, next_state, 'move')
        )

    def send_joint_command(self, joint_positions, next_state):
        """Send a MoveGroup goal to move to named joint positions."""
        if not self.move_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available')
            self.busy = False
            return

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = 'arm'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3

        # Joint constraints
        constraints = Constraints()
        for joint_name, position in joint_positions.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        future = self.move_action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._action_response_callback(f, next_state, 'joint_move')
        )

    def _action_response_callback(self, future, next_state, action_name):
        """Handle action goal acceptance."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'{action_name} goal was rejected')
            self.busy = False
            return

        self.get_logger().info(f'{action_name} goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._action_result_callback(f, next_state, action_name)
        )

    def _action_result_callback(self, future, next_state, action_name):
        """Handle action result and transition state."""
        result = future.result()
        self.get_logger().info(f'{action_name} completed → transitioning to {next_state.name}')
        self.state = next_state
        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
