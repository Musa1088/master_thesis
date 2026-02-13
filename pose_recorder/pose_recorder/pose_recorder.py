#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import yaml
from datetime import datetime
from tf_transformations import euler_from_quaternion, quaternion_matrix
from sensor_msgs.msg import JointState
import os
from math import pi
import re
import threading
from rclpy.executors import MultiThreadedExecutor
from pose_recorder.utils import deg_to_rad, rad_to_deg

logger = rclpy.logging.get_logger("StateRecorder")



class RobotPoseRecorder(Node):
    def __init__(self):
        super().__init__('robot_pose_recorder')

        self.declare_parameter('robot_names', 'neura_1 ; neura_2')
        self.declare_parameter('base_frames', 'neura_1/world; neura_2/world')
        self.declare_parameter('ee_frames',   'neura_1/flange; neura_2/flange')
        self.declare_parameter('world_frame', 'world_link')
        self.declare_parameter('save_dir',    '/robotic_ws/src/pose_recorder/project_data/poses')
        self.declare_parameter('sequence_dir','/robotic_ws/src/pose_recorder/project_data/sequences')
        self.declare_parameter('relpose_dir', '/robotic_ws/src/pose_recorder/project_data/relative_poses')
        self.declare_parameter('config_dir',  '/robotic_ws/src/pose_recorder/project_data/config')
        self.declare_parameter('pose_name',   '<Insert pose name here>')
        self.declare_parameter('pose_description', '<Insert pose description here>')

        # 1) Grab the full list of topics on the ROS graph:
        all_topics = self.get_topic_names_and_types()

        # 2) Filter for those ending in '/joint_states' and extract the namespace:
        names = []
        for topic_name, _ in all_topics:
            if topic_name.endswith('/dynamic_joint_states'):
                # strip leading slash, split on '/', take the first element
                ns = topic_name.lstrip('/').split('/')[0]
                names.append(ns)

        # 3) Sort to keep order deterministic
        self.robot_names = sorted(names)

        # 4) Fallback to parameter if nothing was found
        if not self.robot_names:
            raw = self.get_parameter('robot_names').get_parameter_value().string_value
            self.robot_names = [
                token for token in re.split(r'[ ,;]+', raw) if token
            ]

        self.get_logger().info(f"Discovered robot namespaces: {self.robot_names}")

        #self.robot_names =  [token for token in re.split(r'[ ,;]+', self.get_parameter('robot_names').get_parameter_value().string_value) if token]
        self.ee_frames =    [token for token in re.split(r'[ ,;]+', self.get_parameter('ee_frames').get_parameter_value().string_value) if token]
        self.base_frames =  [token for token in re.split(r'[ ,;]+', self.get_parameter('base_frames').get_parameter_value().string_value) if token]

        self.num_robots = len(self.robot_names)
        self.save_dir = self.get_parameter('save_dir').get_parameter_value().string_value
        self.sequence_dir = self.get_parameter('sequence_dir').get_parameter_value().string_value
        self.relpose_dir = self.get_parameter('relpose_dir').get_parameter_value().string_value
        self.config_dir = self.get_parameter('config_dir').get_parameter_value().string_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.pose_name = self.get_parameter('pose_name').get_parameter_value().string_value
        self.pose_description = self.get_parameter('pose_description').get_parameter_value().string_value

        # Create robot config dict
        self.robot_config = {
            'robot_names': self.robot_names,
            'world_frame': self.world_frame
        }

        for robot_index, robot_name in enumerate(self.robot_names):
            self.robot_config[robot_name] = {
                'base_frame':  self.base_frames[robot_index],
                'ee_frame':    self.ee_frames[robot_index],
                'gripper': None
            } 
        
        #logger.info(f"Robot configuration: {self.robot_config}")
        #logger.info(f"Detected robot names: {self.robot_names}")

        self.latest_joint_states = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        for robot_name in self.robot_names:

            joint_states_topic = f"{robot_name}/joint_states"
            self.subscription = self.create_subscription(
                JointState,
                joint_states_topic,
                self.joint_state_callback,
                10
            )
            logger.info(f"Subscribed to {joint_states_topic}")
        
        logger.info(f"🔍 Reading pose for robots {self.robot_names}")

        self.first_js_message_received = {}
        for robot in self.robot_names:
            self.first_js_message_received[robot] = False
    
    def _lookup_transform(self, source_frame, target_frame):
        try:
            return self.tf_buffer.lookup_transform(
                source_frame,      # source frame
                target_frame,      # target frame
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return -1

    
    def joint_state_callback(self, msg):

        # Determine which robot this message belongs to
        detected_robot = None
        for robot in self.robot_names:
            if any(name.startswith(robot) for name in msg.name):
                filtered_names = []
                filtered_positions = []
                for name, pos in zip(msg.name, msg.position):
                    if 'cartesian' not in name:
                        filtered_names.append(name)
                        filtered_positions.append(pos)

                clean_msg = JointState()
                clean_msg.header = msg.header
                clean_msg.name = filtered_names
                clean_msg.position = filtered_positions
                #logger.info(f"Received joint state for {robot}:")
                self.latest_joint_states[robot] = clean_msg
                self.first_js_message_received[robot] = True
                return


    def _generate_header(self, which_robot):
        """
        Generate a header for the pose data.
        """

        header = {
            'pose_name':    self.pose_name,
            'description':  self.pose_description,
            'recorded':     datetime.now().strftime("%d.%m.%Y %H:%M:%S"),
            'last_edited':  datetime.now().strftime("%d.%m.%Y %H:%M:%S"),
        }
        robot_number = 0
        for idx, robot_name in enumerate(self.robot_names):
            if which_robot & (1 << idx):
                robot_number += 1
                header[f'robot_{robot_number}'] = robot_name

        return header
    

    def _get_joint_state(self, robot_name: str) -> dict:
        """
        Get the latest joint state for a specific robot, with joints
        sorted by their numeric index (joint1, joint2, ...).
        """
        logger.info(f"Bak burasiiii {self.latest_joint_states}" )
        msg = self.latest_joint_states[robot_name]
        

        # Pair up (raw_name, position_rad)
        pairs = list(zip(msg.name, msg.position))

        # Extract the numeric index from names like '...joint3'
        def idx_of(name: str) -> int:
            m = re.search(r'(\d+)$', name)
            return int(m.group(1)) if m else float('inf')

        # Sort by that index
        pairs.sort(key=lambda na: idx_of(na[0]))

        # Split back into two lists, in order
        ordered_names = [n for n, _ in pairs]
        ordered_rads  = [round(p, 6) for _, p in pairs]
        ordered_degs  = [round(rad_to_deg(p), 6) for p in ordered_rads]

        joint_angles_deg = {
            f'joint_{idx_of(n)}': d
            for n, d in zip(ordered_names, ordered_degs)
        }

        return {
            'joint_names': ordered_names,
            'joint_angles_degrees': joint_angles_deg,
        }
    
    def transform_to_matrix(self, tf_msg):
        """
        Convert a TransformStamped message to a 4x4 transformation matrix.
        """
        trans = tf_msg.transform.translation
        rot = tf_msg.transform.rotation

        # Get the rotation matrix from quaternion
        q = [rot.x, rot.y, rot.z, rot.w]
        T = quaternion_matrix(q)  # returns a 4x4 matrix

        # Insert translation
        T[0, 3] = trans.x
        T[1, 3] = trans.y
        T[2, 3] = trans.z

        return T
    
    def _get_transformation_matrix(self, frame_1: str, frame_2: str) -> dict:
        tf: TransformStamped = self._lookup_transform(frame_1, frame_2)
        T_matrix = self.transform_to_matrix(tf)
        logger.info(f"Transformation Matrix:\n {T_matrix} ")
        matrix_list = [[round(val, 7) for val in row] for row in T_matrix.tolist()]  # Convert to nested list
        for robot_name in self.robot_names:
            if frame_1.startswith(robot_name + '/') or frame_1.startswith(robot_name + '_'):
                # drop the “robot_name/” prefix
                frame_1 = frame_1[len(robot_name) + 1:]
            if frame_2.startswith(robot_name + '/') or frame_2.startswith(robot_name + '_'):
                # drop the “robot_name/” prefix
                frame_2 = frame_2[len(robot_name) + 1:]
        return {
            'parent_frame': frame_1,
            'child_frame': frame_2,
            'tf_matrix': matrix_list                       
        }

    def _get_relative_ee_pose(self, frame_1: str, frame_2: str) -> dict:
        """
        Get the latest cartesian pose for a specific robot.
        Parameters
        ----------
        robot_name : str
            The name of the robot.
        Returns
        -------
        dict
            The latest cartesian pose message for the robot.
        """
        logger.info(f'robot1: {frame_1}  robot2: {frame_2}')
        tf: TransformStamped = self._lookup_transform(frame_1, frame_2)

        # Extract rotation and convert quaternion to Euler angles
        rot = tf.transform.rotation

        roll_rad, pitch_rad, yaw_rad = euler_from_quaternion(
            [rot.x, rot.y, rot.z, rot.w]
        )
        # Convert to degrees
        roll_deg = rad_to_deg(roll_rad)
        pitch_deg = rad_to_deg(pitch_rad)
        yaw_deg = rad_to_deg(yaw_rad)

        for robot_name in self.robot_names:
            if frame_1.startswith(robot_name + '/') or frame_1.startswith(robot_name + '_'):
                # drop the “robot_name/” prefix
                frame_1_filtered = frame_1[len(robot_name) + 1:]
            if frame_2.startswith(robot_name + '/') or frame_2.startswith(robot_name + '_'):
                # drop the “robot_name/” prefix
                frame_2_filtered = frame_2[len(robot_name) + 1:]

        return {'relative_cartesian_pose': {
                    'parent_frame_name': frame_1_filtered,
                    'child_frame_name': frame_2_filtered,
                    'translation_m': {
                    'x': round(float(tf.transform.translation.x),4),
                    'y': round(float(tf.transform.translation.y),4),
                    'z': round(float(tf.transform.translation.z),4)
                    },
                    'rotation_euler_degrees': {
                        'roll': round(roll_deg, 1),
                        'pitch': round(pitch_deg, 1),
                        'yaw': round(yaw_deg, 1)
                    },
                    'transformation_matrix': self._get_transformation_matrix(frame_1, frame_2)['tf_matrix']  
                }
    }

    def _get_relative_pose(self, frame_A: str, frame_B: str) -> dict:
        """
        Get the latest cartesian pose for a specific robot.
        Parameters
        ----------
        robot_name : str
            The name of the robot.
        Returns
        -------
        dict
            The latest cartesian pose message for the robot.
        """

        tf: TransformStamped = self._lookup_transform(frame_A, frame_B)

        # Extract rotation and convert quaternion to Euler angles
        rot = tf.transform.rotation

        roll_rad, pitch_rad, yaw_rad = euler_from_quaternion(
            [rot.x, rot.y, rot.z, rot.w]
        )
        # Convert to degrees
        roll_deg = rad_to_deg(roll_rad)
        pitch_deg = rad_to_deg(pitch_rad)
        yaw_deg = rad_to_deg(yaw_rad)

        return {'relative_cartesian_pose': {
                    'parent_frame_name': frame_A,
                    'child_frame_name': frame_B,
                    'ee_translation_m': {
                        'x': round(float(tf.transform.translation.x),4),
                        'y': round(float(tf.transform.translation.y),4),
                        'z': round(float(tf.transform.translation.z),4)
                    },
                    'ee_rotation_euler_degrees': {
                        'roll':     round(roll_deg, 1),
                        'pitch':    round(pitch_deg, 1),
                        'yaw':      round(yaw_deg, 1)
                    }
                }
    }
    
    
    def _get_cartesian_pose(self, robot_name: str) -> dict:
        """
        Get the latest cartesian pose for a specific robot.
        Parameters
        ----------
        robot_name : str
            The name of the robot.
        Returns
        -------
        dict
            The latest cartesian pose message for the robot.
        """

        base_frame = self.robot_config[robot_name]['base_frame']  
        ee_frame = self.robot_config[robot_name]['ee_frame']
        logger.info(f'base: {base_frame},  ee_frame: {ee_frame}, robot_name: {robot_name}')

        tf: TransformStamped = self._lookup_transform(base_frame, ee_frame)
        # Extract rotation and convert quaternion to Euler angles
        rot = tf.transform.rotation

        roll_rad, pitch_rad, yaw_rad = euler_from_quaternion(
            [rot.x, rot.y, rot.z, rot.w]
        )

        # Convert to degrees
        roll_deg = rad_to_deg(roll_rad)
        pitch_deg = rad_to_deg(pitch_rad)
        yaw_deg = rad_to_deg(yaw_rad)

        for robot_name in self.robot_names:
            if base_frame.startswith(robot_name + '/') or base_frame.startswith(robot_name + '_'):
                # drop the “robot_name/” prefix
                base_frame = base_frame[len(robot_name) + 1:]
            if ee_frame.startswith(robot_name + '/') or ee_frame.startswith(robot_name + '_'):
                # drop the “robot_name/” prefix
                ee_frame = ee_frame[len(robot_name) + 1:]


        return {'base_frame_name': base_frame,
                'ee_frame_name': ee_frame,
                'ee_translation_m': {
                    'x': round(float(tf.transform.translation.x),4),
                    'y': round(float(tf.transform.translation.y),4),
                    'z': round(float(tf.transform.translation.z),4)
                    },
                'ee_rotation_euler_degrees': {
                        'roll': round(roll_deg, 1),
                        'pitch': round(pitch_deg, 1),
                        'yaw': round(yaw_deg, 1)
                    }
                
        }
    
    
    def save_pose(self, pose_data: dict):
        """
        Save the pose data to a YAML file.

        Parameters
        ----------
        pose_data : dict
            The pose data to save.
        """

        # Create the directory if it doesn't exist
        script_dir = os.path.dirname(os.path.abspath(__file__))
        save_dir = os.path.join(script_dir, self.save_dir)
        os.makedirs(save_dir, exist_ok=True)

        #timestamp = datetime.now().strftime("%d.%m.%Y__%H:%M:%S")
        filename = f'{self.pose_name}.yaml'

        save_path_filename = os.path.join(save_dir, filename)

        with open(save_path_filename, 'w') as f:
            yaml.dump(pose_data, f, sort_keys=False)

        self.get_logger().info(f"✅ Robot pose saved to {save_path_filename}")

    
    
    def lookup_pose(self, which_robot):
        """
        which_robot is a bitmask:
        bit 0 (1) → include robot_1
        bit 1 (2) → include robot_2
        bit 2 (4) → include robot_3
        …
        """
        # 1) Wait for at least one JS message per robot
        if not all(self.first_js_message_received.values()):
            return

        pose_data = {}
        # 2) Header  
        pose_data['header'] = self._generate_header(which_robot)

        # 3) Gather per-robot poses
        selected_indices = []
        robot_number = 0
        joint_pose = {}
        for idx, robot_name in enumerate(self.robot_names):
            joint_state = self._get_joint_state(robot_name)
            if which_robot & (1 << idx):
                robot_number += 1
                selected_indices.append(idx)
                key = f'robot_{robot_number}'
                #joint_pose['joint_angles_degrees'] = joint_state['joint_angles_degrees'] 
                logger.info(f'Collecting pose for {robot_name}')
                pose_data[key] = {
                    'joint_pose':     {'joint_angles_degrees': joint_state['joint_angles_degrees']},
                    'cartesian_pose': self._get_cartesian_pose(robot_name)
                }

        return pose_data


def main():
    rclpy.init()
    node = RobotPoseRecorder()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
