import copy
import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath, GetStateValidity
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import MoveItErrorCodes, Constraints, JointConstraint, DisplayTrajectory, RobotTrajectory
from moveit_msgs.action import ExecuteTrajectory
from control_msgs.action import FollowJointTrajectory
import itertools
from pose_recorder.utils import deg_to_rad
import time



import rclpy
import copy
from collections import defaultdict

logger = rclpy.logging.get_logger("joint_traj_commander")

class TrajectoryCommander:
    """
    Plans+executes a joint‐goal on the 'combined' MoveIt group via /move_action.
    """

    def __init__(self, node, group_name: str = "combined"):
        self.node = node
        self.logger = node.get_logger()
        self._done = False
        self._current_goal_handle = None
        # --- in __init__ add ---
        self._exec_goal_handle = None
        self._active_follow_handles: list = []
        self._cancelled = False
        # MoveGroup action client
        self._client = ActionClient(node, MoveGroup, '/move_action')

        self._validity_client = self.node.create_client(GetStateValidity, "/check_state_validity")

        # add Cartesian‐path service client
        self._cart_client = node.create_client(GetCartesianPath, '/compute_cartesian_path')
        while not self._cart_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("Waiting for /compute_cartesian_path…")
        
        # ExecuteTrajectory action client
        self._exec_client = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')

        self.traj_pub = node.create_publisher(JointTrajectory,
                                            '/neura_1/scaled_trajectory__joint_pos/joint_trajectory',
                                            qos_profile=10)

        # publish to the standard RViz display topic
        self._display_pub = node.create_publisher(
            DisplayTrajectory,
            '/display_planned_path',
            10
        )

        # publish custom goal states for RViz (External Comm. must be enabled)
        self._goal_state_pub = node.create_publisher(
            RobotState,
            '/rviz/moveit/update_custom_goal_state',
            10
        )

        # create dict for planning parameters
        self.parameters = {
            'planning_group':                   group_name,
            'allowed_planning_time':            5.0,
            'max_velocity_scaling_factor':      0.1,
            'max_acceleration_scaling_factor':  0.5,
            'replan':                           True,
            'replan_attempts':                  10,
            'replan_delay':                     0.0,
            'plan_only':                        True,
            'look_around':                      False,
            'publish_display':                  True,
            'enable_robot_1':                   True,
            'enable_robot_2':                   True,
            'planner_type':                    "PTP"  # Default planner type
        }
    
    def _set_parameters(self, parameters):
        self.parameters = parameters
        logger.info("Parameter update received!")
        self._log_planning_parameters()
    
    def _log_planning_parameters(self):
        """
        Print current motion‐planning parameters to the ROS2 log in a structured way.
        """
        logger.info("=== Motion Planning Parameters ===")
        for key, value in self.parameters.items():
            logger.info(f"  {key:<30}: {value}")
        logger.info("==================================")


    def cancel(self):
        """Cancel any in-flight goals and attempt immediate stop."""
        self.logger.info("Cancelling any active goals")
        self._cancelled = True

        # Cancel MoveGroup goal
        if getattr(self, '_current_goal_handle', None):
            try:
                self._current_goal_handle.cancel_goal_async()
            except Exception as e:
                self.logger.warning(f"Failed cancelling MoveGroup goal: {e}")

        # Cancel ExecuteTrajectory goal (cartesian single)
        if getattr(self, '_exec_goal_handle', None):
            try:
                self._exec_goal_handle.cancel_goal_async()
            except Exception as e:
                self.logger.warning(f"Failed cancelling ExecuteTrajectory goal: {e}")

        # Cancel all FollowJointTrajectory goals
        for handle in list(getattr(self, '_active_follow_handles', [])):
            try:
                handle.cancel_goal_async()
            except Exception as e:
                self.logger.warning(f"Failed cancelling FollowJointTrajectory goal: {e}")

        # Clear stored handles to avoid reuse
        self._current_goal_handle = None
        self._exec_goal_handle = None
        self._active_follow_handles.clear()

        # Treat cancellation as done so callers unblock
        self._done = True

    def reset(self):
        """Clear the done flag before a new plan+exec."""
        self._done = False
        self._cancelled = False
        self._current_goal_handle = None
        self._exec_goal_handle = None
        self._active_follow_handles.clear()

    def is_done(self) -> bool:
        """True once the move_action has completed."""
        return self._done
    
    def send_trajectory(self, joint_names: list[str], positions: list[float], waypoints: list[Pose], robots: list[str], header,):
        if 'PTP' in self.parameters['planner_type']:
            self.send_ptp_trajectory(joint_names, positions)
        elif 'Linear' in self.parameters['planner_type']:
            self.send_linear_trajectory(waypoints, robots, header)

    def send_ptp_trajectory(self,
                        joint_names: list[str],
                        positions: list[float]):
        """Plan+execute a single‐point joint trajectory on the combined group."""
        # wait for MoveGroup action server
        if not self._client.wait_for_server(timeout_sec=2.0):
            self.logger.error("move_action server not available")
            return

        # Build a single‐point joint constraint
        jc = []
        for name, pos in zip(joint_names, positions):
            c = JointConstraint()
            c.joint_name = name
            c.position = pos
            c.tolerance_above = 0.0
            c.tolerance_below = 0.0
            c.weight = 1.0
            jc.append(c)
        
        goal = MoveGroup.Goal()

        # request planning for our 'combined' group
        goal.request.group_name = self.parameters['planning_group'] 

         # --- set maximum planning time ---
        goal.request.allowed_planning_time = self.parameters['allowed_planning_time'] 
        goal.request.max_velocity_scaling_factor = self.parameters['max_velocity_scaling_factor'] 
        goal.request.max_acceleration_scaling_factor = self.parameters['max_acceleration_scaling_factor'] 
        
        # tell it to plan+execute in one shot
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        
        # set number of replan attempts ---
        goal.planning_options.replan = self.parameters['replan'] 
        goal.planning_options.replan_attempts = self.parameters['replan_attempts'] 
        goal.planning_options.replan_delay = self.parameters['replan_delay'] 

        # convert our joint constraints into a Constraints message
        cons = Constraints()
        cons.joint_constraints = jc
        goal.request.goal_constraints = [cons]

        # reset done flag
        self.reset()

        # STEP 1: PLAN ONLY
        plan_goal = copy.deepcopy(goal)
        plan_goal.planning_options.plan_only = True
        plan_future = self._client.send_goal_async(plan_goal)
        plan_future.add_done_callback(
            lambda fut: self._on_ptp_plan_response(fut, goal, joint_names, positions)
        )

    
    def publish_goal_state(self,
                           joint_names: list[str],
                           positions: list[float]):
        # Publish the Goal state to RViz
        rs = RobotState()
        rs.joint_state = JointState()
        rs.joint_state.name = joint_names
        rs.joint_state.position = positions
        self._goal_state_pub.publish(rs)
        self.logger.info("Published custom goal RobotState for RViz")

    def _on_ptp_plan_response(self, future, exec_goal, joint_names, positions):
        
        goal_handle = future.result()
        self._current_goal_handle = goal_handle
        if not goal_handle.accepted:
            self.logger.error("planning request was rejected")
            return

        # now request the actual result
        goal_handle.get_result_async().add_done_callback(
            lambda res_fut: self._on_ptp_plan_result(res_fut, exec_goal)
        )

    def _on_ptp_plan_result(self, future, exec_goal):

        if self._cancelled:
            return

        # future.result() is a MoveGroup_GetResult_Response
        response = future.result()
        result = response.result
        error_code = result.error_code.val

        if error_code != MoveItErrorCodes.SUCCESS:
            self.logger.error(f"planning failed ({error_code})")
            return

        # publish the planned trajectory for the orange ghost
        disp = DisplayTrajectory()
        disp.model_id = "maira7M"  
        disp.trajectory_start = result.trajectory_start
        disp.trajectory.append(result.planned_trajectory)
        self._display_pub.publish(disp)
        logger.info("Planned path was published")

        # STEP 2: EXECUTE
        exec_goal.planning_options.plan_only = False
        exec_future = self._client.send_goal_async(exec_goal)
        #exec_future.add_done_callback(self._on_exec_response)
        def store_and_handle_exec(fut):
            goal_handle = fut.result()
            self._current_goal_handle = goal_handle
            self._on_exec_response(fut)
        exec_future.add_done_callback(store_and_handle_exec)
    
    
    def _on_exec_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.error("execute_trajectory goal rejected")
            return
        goal_handle.get_result_async().add_done_callback(self._on_exec_result)

    def _on_exec_result(self, future):
        result = future.result().result
        self._done = True #(result.error_code.val == MoveItErrorCodes.SUCCESS)
        if self._done:
            self.logger.info("trajectory execution succeeded")
        else:
            self.logger.warn(f"trajectory execution failed ({result.error_code.val})")


    def send_linear_trajectory(self,
                           waypoints: list[Pose],
                           robots: list[str],
                           header,
                           eef_step: float = 0.01,
                           jump_threshold: float = 0.0,
                           avoid_collisions: bool = True):
        """
        Plan and execute synchronized Cartesian paths for multiple robots.
        """
        self._cartesian_responses = {}
        self._cartesian_expected = len(robots)
        self.reset()

        def cartesian_done(idx, section):
            def _cb(fut):
                if self._cancelled:
                    return
                resp = fut.result()
                if resp.error_code.val != MoveItErrorCodes.SUCCESS:
                    self.logger.error(f"{section} Cartesian planning failed: {resp.error_code.val}")
                    self._cartesian_responses[section] = None
                else:
                    self._cartesian_responses[section] = resp.solution
                # When all responses are in, merge and execute
                if len(self._cartesian_responses) == self._cartesian_expected:
                    self._merge_and_execute_cartesian(robots)
            return _cb

        for idx, section in enumerate(robots):
            joint_pose = self.node._get_joint_state(section)
            joint_angles_deg = joint_pose['joint_angles_degrees'].values()
            request = GetCartesianPath.Request()
            request.header = header[idx]
            request.start_state = RobotState(
                joint_state=JointState(
                    name=joint_pose['joint_names'],
                    position=deg_to_rad(list(angle for angle in joint_angles_deg))
                )
            )
            request.group_name = f'{section}_arm'  # Use correct group name!
            #request.group_name = 'combined'  # Use correct group name!
            request.link_name = f'{section}/flange'
            request.waypoints = waypoints[idx]
            request.max_step = eef_step
            request.jump_threshold = jump_threshold
            request.avoid_collisions = avoid_collisions
            request.max_velocity_scaling_factor = self.parameters['max_velocity_scaling_factor']
            request.max_acceleration_scaling_factor = self.parameters['max_acceleration_scaling_factor']
            self._cart_client.call_async(request).add_done_callback(cartesian_done(idx, section))

    def _merge_and_execute_cartesian(self, robots):

        if self._cancelled:
            return
        
        # Only proceed if all succeeded
        if any(self._cartesian_responses[section] is None for section in robots):
            self.logger.error("At least one Cartesian plan failed, aborting combined execution.")
            return

        # Extract trajectories
        trajs = [self._cartesian_responses[section].joint_trajectory for section in robots]

        # Merge joint names
        combined = JointTrajectory()
        combined.joint_names = []
        joint_angles = []
        for jt in trajs:
            combined.joint_names.extend(jt.joint_names)
            joint_angles.extend(jt.points[-1].positions)

        self.publish_goal_state(list(combined.joint_names), list(joint_angles))

        # Directly zip points (assumes same length and matching time_from_start)
        for pts in zip(*[jt.points for jt in trajs]):
            pt = JointTrajectoryPoint()
            pt.time_from_start = pts[0].time_from_start  # all are the same
            pt.positions = []
            for p in pts:
                pt.positions.extend(p.positions)
            combined.points.append(pt)

        # ✅ INSERT VALIDATION HERE
        if not self._validate_trajectory_collision_free(combined):
            self.logger.error("Trajectory validation failed due to collision.")
            return
    
            # Send each trajectory to its respective controller and track completion
        robots_done: dict[str, bool | None] = {section: None for section in robots}

        def check_all_done():
            if all(v is not None for v in robots_done.values()):
                # mark overall done
                self._done = True
                if all(robots_done.values()):
                    self.logger.info("All arm trajectories completed successfully.")
                else:
                    self.logger.warning("One or more arm executions failed.")

            # Send combined trajectory
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = combined

        # Instead of sending to /combined_controller/follow_joint_trajectory,
        # send each trajectory to its respective controller:
        for section, traj in zip(robots, trajs):
            action_ns = f"/{section}/scaled_trajectory__joint_pos/follow_joint_trajectory"
            client = ActionClient(self.node, FollowJointTrajectory, action_ns)
            while not client.wait_for_server(timeout_sec=2.0):
                self.logger.info(f"Waiting for {action_ns}…")
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            send_fut = client.send_goal_async(goal)

            def make_follow_callbacks(section):
                def goal_response(fut):
                    if self._cancelled:
                        robots_done[section] = False
                        check_all_done()
                        return
                    goal_handle = fut.result()
                    if not goal_handle.accepted:
                        self.logger.error(f"{section}: FollowJointTrajectory goal rejected")
                        robots_done[section] = False
                        check_all_done()
                        return
                    # store handle for possible cancellation
                    self._active_follow_handles.append(goal_handle)

                    def result_cb(res_fut):
                        if self._cancelled:
                            robots_done[section] = False
                            check_all_done()
                            return
                        result = res_fut.result().result
                        # Assume success if error_code == 0, otherwise log failure
                        success = True
                        if hasattr(result, "error_code"):
                            success = (result.error_code == 0)
                        if success:
                            self.logger.info(f"{section}: trajectory execution succeeded")
                        else:
                            self.logger.warn(f"{section}: trajectory execution failed ({getattr(result, 'error_code', 'unknown')})")
                        robots_done[section] = success
                        check_all_done()
                    goal_handle.get_result_async().add_done_callback(result_cb)
                return goal_response

            send_fut.add_done_callback(make_follow_callbacks(section))

    def _validate_trajectory_collision_free(self, combined_trajectory: JointTrajectory) -> bool:
        if not self._validity_client.wait_for_service(timeout_sec=2.0):
            self.logger.error("GetStateValidity service not available.")
            return False

        for idx, point in enumerate(combined_trajectory.points):
            state = RobotState()
            state.joint_state.name = combined_trajectory.joint_names
            state.joint_state.position = point.positions

            req = GetStateValidity.Request()
            req.robot_state = state
            req.group_name = "combined"  # or use one of your robot groups, depending

            fut = self._validity_client.call_async(req)
            # Wait for result non-blockingly
            timeout_sec = 2.0
            poll_interval = 0.01
            waited = 0.0
            while not fut.done() and waited < timeout_sec:
                time.sleep(poll_interval)
                waited += poll_interval

            if not fut.done():
                self.logger.warn(f"Timeout waiting for collision check at index {idx}")
                return False

            if not fut.result().valid:
                self.logger.warn(f"State invalid at trajectory index {idx}")
                return False

        self.logger.info("Trajectory validated as collision-free.")
        return True

