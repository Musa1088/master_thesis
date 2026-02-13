import time
import yaml
from math import pi
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from std_msgs.msg import Header
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import subprocess
import os


import rclpy
from typing import Optional, Any, List, Dict
from typing import List, Optional
from rcl_interfaces.srv import SetParametersAtomically
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType

from typing import Any

#yash wussup
logger = rclpy.logging.get_logger("utils")


from typing import List

def kill_nodes_by_keyword(keyword: str):
    try:
        subprocess.run(["pkill", "-f", keyword], check=False)
        logger.info(f"Terminated all processes matching '{keyword}'")
    except Exception as e:
        logger.error(f"Failed to kill processes matching '{keyword}': {e}")


def kill_node_by_name(node_name: str):
    """
    Best‑effort terminate only the servo_node processes matching the fully‑qualified node name
    (e.g. "/neura_2/servo_node") or a bare "/servo_node", and remove it from the dropdown.
    """

    text = node_name.lstrip('/')
    parts = text.split('/', 1)
    if len(parts) == 2:
        ns, nod = parts
        # match exactly that node and namespace
        pattern = f"__node:={nod}.*__ns:=/{ns}"
    else:
        nod = parts[0]
        # match node name anywhere
        pattern = f"__node:={nod}\\b"

    try:
        subprocess.run(["pkill", "-f", pattern], check=False)
        logger.info(f"Terminated node: {node_name}")
    except Exception as e:
        logger.error(f"Failed to kill node '{node_name}': {e}")
        return

def get_node_names(
    node_obj,
    keyword: str,
    *,
    include_namespace: bool = True,
    prefix_slash: bool = True,
    ignore_case: bool = False,
    exclude_keywords: list[str] | None = None,
    filter_duplicates: bool = True,
) -> list[str]:
    """
    Return fully qualified node names (e.g. '/namespace/node_name') for all running
    nodes whose name contains `keyword` and does NOT contain any of the `exclude_keywords`.
    If `filter_duplicates` is True, only the first occurrence of each node name is kept
    (regardless of namespace); otherwise duplicates are filtered by the final output string.
    """
    try:
        raw = node_obj.get_node_names_and_namespaces()
    except Exception:
        return []

    def contains(hay: str, sub: str) -> bool:
        if ignore_case:
            return sub.lower() in hay.lower()
        return sub in hay

    results = []
    seen = set()

    for name, ns in raw:
        if not contains(name, keyword):
            continue

        if include_namespace and ns:
            base = f"{ns}/{name}".lstrip('/')
        else:
            base = name

        if exclude_keywords:
            skip = False
            for ex in exclude_keywords:
                if contains(base, ex):
                    skip = True
                    break
            if skip:
                continue

        fq = f"/{base}" if prefix_slash else base
        if filter_duplicates and fq in seen:
            continue
        seen.add(fq)
        results.append(fq)

    return results

def normalize_frame_name(s: str) -> str:
    """
    Turn strings like "parent: 'world_link'" or 'child_frame_id: "foo"' or '  world_link  '
    into the bare frame name, e.g. "world_link".
    """
    val = s.strip()
    if ':' in val:
        _, val = val.split(':', 1)
    val = val.strip()
    # strip surrounding single or double quotes
    if len(val) >= 2 and ((val[0] == val[-1] == "'") or (val[0] == val[-1] == '"')):
        val = val[1:-1]
    # remove trailing colons or whitespace leftovers
    return val.rstrip(':').strip()


def get_tf_frames(
    node,
    namespaces: List[str],
    *,
    whitelist: Optional[List[str]] = None,
    override_keywords: Optional[List[str]] = None,
) -> List[str]:
    if not hasattr(node, 'tf_buffer'):
        return []

    try:
        raw_frames = list(node.tf_buffer._buffer.get_frame_strings())
        frames = [f.decode() if isinstance(f, bytes) else str(f) for f in raw_frames]
    except Exception:
         yaml_str = node.tf_buffer.all_frames_as_yaml()
         frames = [line.strip().rstrip(':') for line in yaml_str.splitlines() if line.strip()]

    filtered = []
    for ns in namespaces:
        ns_frames = [f for f in frames if f.startswith(ns)]
        if whitelist:
            ns_frames = [f for f in ns_frames if any(w in f for w in whitelist)]
        filtered.extend(ns_frames)

    if override_keywords:
        for kw in override_keywords:
            for f in frames:
                if kw in f and f not in filtered:
                    filtered.append(normalize_frame_name(f))

    # dedupe, preserving order
    seen = set()
    result = []
    for f in filtered:
        if f not in seen:
            seen.add(f)
            result.append(f)
    return result



def get_node_parameters(node_name: str, parameters: List[str], timeout_sec: float = 2.0
) -> dict[str, Any] | None:
    """
    Synchronously fetch a set of parameters from a running ROS2 node.

    Args:
        node_name: the fully‑qualified node name (with or without leading '/')
        parameter_names: list of parameter keys to retrieve
        timeout_sec: how long to wait for service & response

    Returns:
        A dict mapping each requested name to its value, or None on error/timeout.
    """
    # Initialize ROS if needed
    if not rclpy.ok():
        rclpy.init()

    # Create a temporary node just for this call
    get_param_node = Node('param_getter')
    service_name = f"/{node_name.lstrip('/')}/get_parameters"
    client = get_param_node.create_client(GetParameters, service_name)

    # Wait for service to come up
    if not client.wait_for_service(timeout_sec=timeout_sec):
        get_param_node.get_logger().error(f"Service {service_name} unavailable after {timeout_sec}s")
        get_param_node.destroy_node()
        return None

    # Build request
    req = GetParameters.Request()
    req.names = parameters
    get_param_node.get_logger().info(f"Requesting {parameters} from {service_name}")

    # Call async
    future = client.call_async(req)

    # Spin until done or timeout
    executor = SingleThreadedExecutor()
    executor.add_node(get_param_node)
    status = executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
    executor.remove_node(get_param_node)

    if not future.done():
        get_param_node.get_logger().error(f"Timed out waiting for {service_name}")
        get_param_node.destroy_node()
        return None


    try:
        resp = future.result()
    except Exception as e:
        get_param_node.get_logger().error(f"Service call to {service_name} failed: {e}")
        return None
    

    result: Dict[str, Any] = {}
    for name, val in zip(parameters, resp.values):
        if val.type == ParameterType.PARAMETER_BOOL:
            result[name] = val.bool_value
        elif val.type == ParameterType.PARAMETER_INTEGER:
            result[name] = val.integer_value
        elif val.type == ParameterType.PARAMETER_DOUBLE:
            result[name] = val.double_value
        elif val.type == ParameterType.PARAMETER_STRING:
            result[name] = val.string_value
        else:
            get_param_node.get_logger().warning(f"Unsupported type {val.type} for '{name}', skipping")
    
    get_param_node.destroy_node()
    
    return result



def set_node_parameters(node_name: str, parameters: dict[str, Any], exec_node) -> None:
        """
        Sets the parameters for a specific node.

        Args:
            node_name (str): The name of the node.
            parameters (dict[str, Any]): A dictionary containing the parameters to be set.
            Key as the parameter name and value as the parameter value. Allowed are, Int, Bool, Float, and String.
            exec_node: node handle to execute the service request
        Returns:
            None
        """
        service_name = f"/{node_name}/set_parameters_atomically"
        req = SetParametersAtomically.Request()
        for key, value in parameters.items():
            param = Parameter()
            param.name = key
            param.value = ParameterValue()
            if isinstance(value, bool):
                param.value.type = 1
                param.value.bool_value = value
            elif isinstance(value, int):
                param.value.type = 2
                param.value.integer_value = value
            elif isinstance(value, float):
                param.value.type = 3
                param.value.double_value = value
            elif isinstance(value, str):
                param.value.type = 4
                param.value.string_value = value
            else:
                logger.error(
                    f"Value {value} of type {type(value)} is not supported for parameter {key}. Skipping."
                )
                continue
            req.parameters.append(param)
        client = exec_node.create_client(SetParametersAtomically, service_name)
        resp = client.call(req).result
        if resp.successful:
            logger.info(
                f"Setting parameters successully for node {node_name}"
            )
        else:
            logger.error(
                f"Setting parameters failed for node {node_name}. Reason: {resp.reason}"
            )



def update_fk(fk_client, joint_angles, section, joint_names, base_frame, ee_frame, callback, pose_data, fk_pose_received):
    """Send a request to the FK service."""
    request = GetPositionFK.Request()
    logger.info(f"End effector frame: {ee_frame}")
    logger.info(f"Base frame: {base_frame}")
    request.fk_link_names = [ee_frame]
    request.header.frame_id = base_frame
    request.robot_state = RobotState()
    request.robot_state.joint_state.name = joint_names
    request.robot_state.joint_state.position = joint_angles

    future = fk_client.call_async(request)
    future.add_done_callback(lambda f: callback(f, section, base_frame, ee_frame, pose_data, fk_pose_received))

def update_ik(ik_client, pose_data, section, robot_name, base_frame, ee_frame, joint_angles_rad, joint_names, callback, file_path):
    """Send a request to the IK service."""
    pose_dict = pose_data[section]['cartesian_pose']
    position = pose_dict['ee_translation_m']
    rotation = pose_dict['ee_rotation_euler_degrees']

    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = base_frame
    pose_stamped.header.stamp = rclpy.time.Time().to_msg()

    pose_stamped.pose.position.x = float(position['x'])
    pose_stamped.pose.position.y = float(position['y'])
    pose_stamped.pose.position.z = float(position['z'])

    qx, qy, qz, qw = quaternion_from_euler(
        deg_to_rad(rotation['roll']), deg_to_rad(rotation['pitch']), deg_to_rad(rotation['yaw'])
    )
    pose_stamped.pose.orientation.x = qx
    pose_stamped.pose.orientation.y = qy
    pose_stamped.pose.orientation.z = qz
    pose_stamped.pose.orientation.w = qw

    request = GetPositionIK.Request()
    request.ik_request.group_name = f"{robot_name}_arm"
    request.ik_request.robot_state = RobotState()
    request.ik_request.ik_link_name = ee_frame
    request.ik_request.pose_stamped = pose_stamped
    request.ik_request.timeout.sec = 2

    if 'joint_pose' in pose_data[section]:
        request.ik_request.robot_state.joint_state.name = joint_names
        request.ik_request.robot_state.joint_state.position = joint_angles_rad

    future = ik_client.call_async(request)
    future.add_done_callback(lambda f: callback(f, section, robot_name, pose_data, file_path))

def handle_fk_result(future, section, base_frame, ee_frame, pose_data, fk_pose_received):
    """Handle the FK service result."""
    try:
        response = future.result()
        if not response.pose_stamped:
            logger.info(f"❗ No pose returned from FK")
            return
        pose = response.pose_stamped[0].pose
        logger.info(f"✅ FK pose: {pose}")
        fk_pose_received.emit(pose, section, base_frame, ee_frame)
    except Exception as e:
        logger.info(f"❗ FK service call failed: {e}")

def handle_ik_result(future, section, robot_name, pose_data, file_path):
    """Handle the IK service result."""
    try:
        response = future.result()
        if response.error_code.val != 1:
            logger.info(f"❗ IK failed with error code: {response.error_code.val}")
            return

        joint_state = response.solution.joint_state
        all_names = joint_state.name
        logger.info(f"✅ IK joint names: {all_names}")
        all_angles = joint_state.position

        filtered = [
            (name, angle)
            for name, angle in zip(all_names, all_angles)
            if name.startswith(f"{robot_name}/joint")
        ]
        filtered_names = [name for name, _ in filtered]
        filtered_angles_rad = [round(angle, 6) for _, angle in filtered]
        filtered_angles_deg = [round(rad * 180 / pi, 6) for rad in filtered_angles_rad]

        pose_data[section]['joint_pose'] = {
            'joint_angles_degrees': {
                f'joint_{name[-1]}': angle for name, angle in zip((filtered_names), filtered_angles_deg)
            },
        }

    except Exception as e:
        logger.info(f"❗ IK service call failed: {e}")

def write_yaml(data, file_path):
    """Write data to a YAML file, forcing the extension to be .yaml."""
    if not file_path.lower().endswith('.yaml'):
        base, _ = os.path.splitext(file_path)
        file_path = base + '.yaml'
    with open(file_path, 'w') as f:
        yaml.dump(data, f, sort_keys=False)
    return file_path


def update_tf_frames(node, parent):
    # Query all TF frames from the buffer
    if not hasattr(node, 'tf_buffer'):
        return

    try:
        # Assumes Buffer.get_frame_strings() is available
        frames = list(node.tf_buffer._buffer.get_frame_strings())
    except Exception:
        yaml_str = node.tf_buffer.all_frames_as_yaml()
        frames = [line.strip().rstrip(':') for line in yaml_str.splitlines() if line.strip()]
    robots = parent.get_robot_list()
    # Limit to two robots
    for idx in range(len(robots)):
        name = robots[idx]
        ee_frame = [f for f in frames if f.startswith(name) and 'flange' in f]
        node.ee_frames[idx] = ee_frame[0] if ee_frame else None
        base_frame = [f for f in frames if f.startswith(name) and f.endswith('world')]
        node.base_frames[idx] = base_frame[0] if base_frame else None
    
    return frames


def rad_to_deg(rad: float | List[float]) -> float | List[float]:
    """
    Convert radians to degrees.

    Parameters
    ----------
    rad : float | List[float]
        The radian(s) to convert.
        Can be a single value or a list of values.

    Returns
    -------
    float | List[float]:
        The converted value(s).
    """
    if isinstance(rad, list):
        return [x * 180 / pi for x in rad]
    return rad * 180 / pi


def deg_to_rad(deg: float | List[float]) -> float | List[float]:
    """
    Convert degrees to radians.

    Parameters
    ----------
    deg : float | List[float]
        The degree(s) to convert.
        Can be a single value or a list of values.

    Returns
    -------
    float | List[float]:
        The converted value(s).
    """
    if isinstance(deg, list):
        return [x * pi / 180 for x in deg]
    return deg * pi / 180

def rmpos(val: float):
    if abs(val) == 0.0:
        return 0.0
    else:
        return val
    
def move_to(node, pose_robot_names, parent, pose_data, robot_enable_checkboxes, traj_commander):
    robot_names = pose_robot_names
    if len(parent.get_robot_list())==1 and len(pose_robot_names) > 1:
        idx = int(parent.get_robot_list()[0].split('_')[-1]) - 1
    else:
        idx = 0
    for _, section in enumerate(parent.get_robot_list()):
        if section in robot_names or (len(parent.get_robot_list()) == 1 and len(pose_robot_names) == 1):
            logger.info(f"Moving to pose for {section}")
            cp = pose_data.get(f'robot_{idx+1}', {}).get('cartesian_pose', {}).get('ee_translation_m')
            if cp:
                logger.info(f"{section} Cartesian translation: x={cp['x']} m, y={cp['y']} m, z={cp['z']} m")
            rot = pose_data.get(f'robot_{idx+1}', {}).get('cartesian_pose', {}).get('ee_rotation_euler_degrees')
            if rot:
                logger.info(f"{section} Euler rotation: roll={rot['roll']}°, pitch={rot['pitch']}°, yaw={rot['yaw']}°") 
            degree = pose_data.get(f'robot_{idx+1}', {}).get('joint_pose', {}).get('joint_angles_degrees', {})
            if degree:
                logger.info(f"{section} Joint angles: {degree} with index {idx}")
            idx += 1
    # Gather both arms’ joint names & positions into one combined trajectory
    all_names = []
    all_positions = []
    waypoints = []
    robots = []
    header = []
    if len(parent.get_robot_list())==1 and len(pose_robot_names) > 1:
        idx = int(parent.get_robot_list()[0].split('_')[-1]) - 1
    else:
        idx = 0
    for _, section in enumerate(parent.get_robot_list()):
        joint_pose = node._get_joint_state(section)
        if section in robot_names or (len(parent.get_robot_list()) == 1 and len(pose_robot_names) == 1):
            logger.info(f"Processing robot section: {robot_names}")
            if robot_enable_checkboxes[section]:
                logger.info(f"Gathering joint angles for {section}")
                jp = pose_data.get(f'robot_{idx+1}', {}).get('joint_pose')
                cp = pose_data.get(f'robot_{idx+1}', {}).get('cartesian_pose')
                if jp:
                    all_names.extend(joint_pose['joint_names'])
                    angles_deg = jp['joint_angles_degrees'].values()
                    all_positions.extend(deg_to_rad(angle) for angle in angles_deg)
                    waypoints.append(waypoint_pose(cp))
                    header.append(Header(frame_id = f'{section}/world'))
                    robots.append(section)
                    logger.info(f"{section} Moveing with Joint angles: {angles_deg} with index {idx}")
                    logger.info(f"Joint names: {joint_pose['joint_names']}")
                idx += 1
            else:
                all_names.extend(joint_pose['joint_names'])
                angles_deg = joint_pose['joint_angles_degrees'].values()
                all_positions.extend(deg_to_rad(angle) for angle in angles_deg) 
                idx += 1              
        else:
            all_names.extend(joint_pose['joint_names'])
            angles_deg = joint_pose['joint_angles_degrees'].values()
            all_positions.extend(deg_to_rad(angle) for angle in angles_deg)
    if not all_names:
        return
    # Reset & send as a single combined goal
    traj_commander.reset()
    traj_commander.publish_goal_state(list(all_names), list(all_positions))
    traj_commander.send_trajectory(
        joint_names=list(all_names),
        positions=list(all_positions),
        waypoints=waypoints,
        robots=robots,
        header=header
    )

def display_target(node, pose_robot_names, parent, pose_data, robot_enable_checkboxes, traj_commander):
    robot_names = pose_robot_names
    if len(parent.get_robot_list())==1:
        idx = int(parent.get_robot_list()[0].split('_')[-1]) - 1
    else:
        idx = 0
    for _, section in enumerate(parent.get_robot_list()):
        if section in robot_names or (len(parent.get_robot_list()) == 1 and len(pose_robot_names) == 1):
            cp = pose_data.get(f'robot_{idx+1}', {}).get('cartesian_pose', {}).get('ee_translation_m')
            if cp:
                logger.info(f"{section} Cartesian translation: x={cp['x']} m, y={cp['y']} m, z={cp['z']} m")
            rot = pose_data.get(f'robot_{idx+1}', {}).get('cartesian_pose', {}).get('ee_rotation_euler_degrees')
            if rot:
                logger.info(f"{section} Euler rotation: roll={rot['roll']}°, pitch={rot['pitch']}°, yaw={rot['yaw']}°")
            
            idx += 1
    # Gather both arms’ joint names & positions into one combined trajectory
    all_names = []
    all_positions = []
    if len(parent.get_robot_list())==1 and len(pose_robot_names) > 1:
        idx = int(parent.get_robot_list()[0].split('_')[-1]) - 1
    else:
        idx = 0
    for _, section in enumerate(parent.get_robot_list()):
        joint_pose = node._get_joint_state(section)
        if section in robot_names or (len(parent.get_robot_list()) == 1 and len(pose_robot_names) == 1):
            if robot_enable_checkboxes[section]:
                logger.info(f"Processing robot section: {section}")
                jp = pose_data.get(f'robot_{idx+1}', {}).get('joint_pose')
                if jp:
                    all_names.extend(joint_pose['joint_names'])
                    angles_deg = jp['joint_angles_degrees'].values()
                    all_positions.extend(deg_to_rad(angle) for angle in angles_deg)
                idx += 1
            else:
                #joint_pose = node._get_joint_state(section)
                all_names.extend(joint_pose['joint_names'])
                angles_deg = joint_pose['joint_angles_degrees'].values()
                all_positions.extend(deg_to_rad(angle) for angle in angles_deg) 
                idx += 1              
        else:
            #joint_pose = node._get_joint_state(section)
            all_names.extend(joint_pose['joint_names'])
            angles_deg = joint_pose['joint_angles_degrees'].values()
            all_positions.extend(deg_to_rad(angle) for angle in angles_deg)

                
    if not all_names:
        return
    # Reset & send as a single combined goal
    #traj_commander.reset()
    traj_commander.publish_goal_state(list(all_names), list(all_positions))

def waypoint_pose(cartesian_pose):
    qx, qy, qz, qw = quaternion_from_euler(
                        deg_to_rad(cartesian_pose['ee_rotation_euler_degrees']['roll']), 
                        deg_to_rad(cartesian_pose['ee_rotation_euler_degrees']['pitch']), 
                        deg_to_rad(cartesian_pose['ee_rotation_euler_degrees']['yaw'])
                    )
    waypoints = [Pose(
                    position=Point(x=cartesian_pose['ee_translation_m']['x'], y=cartesian_pose['ee_translation_m']['y'], z=cartesian_pose['ee_translation_m']['z']),
                    orientation=Quaternion(x =qx, y=qy, z=qz, w=qw))
                ]
    return waypoints
