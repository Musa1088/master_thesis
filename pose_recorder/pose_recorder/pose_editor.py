import os
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R
from math import pi
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox, QFormLayout, QLabel, QLineEdit, QPushButton, QMessageBox,
    QScrollArea, QSizePolicy, QHBoxLayout, QComboBox, QCheckBox, QFileDialog
)
from PySide6.QtCore import Signal
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply, quaternion_from_matrix
from pose_recorder.utils import rad_to_deg, deg_to_rad
from moveit_msgs.srv import GetPositionFK, GetPositionIK
import rclpy
import threading
from concurrent.futures import ThreadPoolExecutor
from pose_recorder.utils import write_yaml, update_fk, handle_fk_result, update_ik, handle_ik_result, deg_to_rad, move_to, display_target
from datetime import datetime
from time import sleep
import copy
from pose_recorder.trajectory_commander import TrajectoryCommander
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from pose_recorder.utils import deg_to_rad, rad_to_deg, get_tf_frames


logger = rclpy.logging.get_logger("pose_editor_gui")


class EditPoseWindow(QWidget):
    _current_instance: "EditPoseWindow" = None
    fk_pose_received = Signal(object, object, object, object)
    pose_renamed = Signal(str, str)  # old_name, new_name

    def __init__(self, file_path, shared_node, parent=None, traj_commander=None):
         # if there’s already an open window, close it
        if EditPoseWindow._current_instance is not None:
            EditPoseWindow._current_instance.close()
        EditPoseWindow._current_instance = self

        super().__init__()
        self.setWindowTitle("Edit Pose")
        self.file_path = file_path
        self.parent = parent  # Reference to the parent widget (e.g., SequenceEditor)
        self.layout = QVBoxLayout(self)
        self.node = shared_node
        self.parent = parent
        self.node_lock = threading.Lock()  # Lock for synchronizing node access
        
        if traj_commander is None:
            self.traj_commander = TrajectoryCommander(self.node)
        else:
            self.traj_commander = traj_commander

        self.fk_client = self.node.create_client(GetPositionFK, '/compute_fk')
        self.ik_client = self.node.create_client(GetPositionIK, '/compute_ik')
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            logger.info("Waiting for FK service...")
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            logger.info("Waiting for IK service...")

        self.solve_fk = False
        self.solve_ik = False
        self.fk_pose_received.connect(self.update_cartesian_pose_fields)

        with open(self.file_path, 'r') as f:
            self.pose_data = yaml.safe_load(f)
        
        self.pose_robot_names = [
                        value
                        for key, value in self.pose_data['header'].items()
                        if key.startswith('robot')
                    ]
        # Copy pose data to be able to reset later, if necessary
        self.initial_pose_data = copy.deepcopy(self.pose_data)

        # Create editor layout
        self.create_layout()

    def closeEvent(self, event):
        if EditPoseWindow._current_instance is self:
            EditPoseWindow._current_instance = None
        super().closeEvent(event)

    def create_layout(self):
        header = self.pose_data.get('header', {})

        # Pose Header
        header_group = QGroupBox("Pose Header")
        header_layout = QFormLayout()
        self.pose_name_edit = QLineEdit()
        self.pose_name_edit.setPlaceholderText("<Insert pose name>")
        self.pose_name_edit.setText(header.get('pose_name', ''))
        header_layout.addRow("Pose name:", self.pose_name_edit)

        self.pose_desc_edit = QLineEdit()
        self.pose_desc_edit.setPlaceholderText("<Insert pose description>")
        self.pose_desc_edit.setText(header.get('description', ''))
        header_layout.addRow("Description:", self.pose_desc_edit)

        header_group.setLayout(header_layout)
        

        self.joint_fields = []
        self.cartesian_fields = []
        self.locked_group = None
        self.fields = {}
        self.ee_frame_combos = {}
        self.ee_frame_display_to_full = {}  
        self.robot_enable_checkboxes = {}   

        h_layout = QHBoxLayout()

        robots = self.parent.get_robot_list()

        tf_whitelist, tf_override_keywords = self.parent.get_tf_filter_params()
        frames = get_tf_frames(
            self.node,
            namespaces=robots,
            whitelist=None,
            override_keywords=tf_override_keywords,
        )

        i = 0
        for i, robot_name in enumerate(self.pose_robot_names):
            logger.info(f"Processing robot robot_name: {robot_name}")
            idx = int(robot_name.split('_')[-1]) - 1  # Extract robot index from robot_name name
            logger.info(f'Nodes robot names {self.pose_robot_names}')
            if robot_name in self.parent.get_robot_list() or (len(self.parent.get_robot_list()) == 1 and len(self.pose_robot_names) == 1):
                if (len(self.parent.get_robot_list()) == 1 and len(self.pose_robot_names) == 1):
                    robot_name = self.parent.get_robot_list()[0]
                content = self.pose_data.get(f'robot_{i+1}', {})
                group_box = QGroupBox(f'robot_{i+1}: {robot_name}')
                group_layout = QVBoxLayout()

                # --- Enable Robot Checkbox ---
                enable_checkbox = QCheckBox("Enable Robot")
                enable_checkbox.setChecked(True)
                self.robot_enable_checkboxes[f'{robot_name}'] = enable_checkbox
                group_layout.addWidget(enable_checkbox)
                # Connect checkbox to lock/unlock fields
                enable_checkbox.stateChanged.connect(
                    lambda state, s=f'robot_{i+1}': self.set_robot_fields_enabled(s, state == 2)
                )
                # --- End Enable Robot Checkbox ---

                # --- EE Frame ComboBox ---
                ee_frames = [f for f in frames if f.startswith(robot_name)]
                # Remove prefix for display
                display_frames = [f[len(robot_name) + 1 :] if f.startswith(robot_name + "_") or f.startswith(robot_name + "/") else f for f in ee_frames]
                # Map display name to full frame name
                self.ee_frame_display_to_full = getattr(self, "ee_frame_display_to_full", {})
                self.ee_frame_display_to_full[f'robot_{i+1}'] = {
                    display: full for display, full in zip(display_frames, ee_frames)
                }

                ee_frame_combo = QComboBox()
                ee_frame_combo.addItems(display_frames)
                # Set current display text
                self.current_ee = content.get('cartesian_pose', {}).get('ee_frame_name', ee_frames[0])

                current_display = self.current_ee[len(robot_name) + 1 :] if self.current_ee.startswith(robot_name + "_") or self.current_ee.startswith(robot_name + "/") else self.current_ee
                if current_display in display_frames:
                    ee_frame_combo.setCurrentText(current_display)
                self.ee_frame_combos[f'robot_{i+1}'] = ee_frame_combo

                # Connect ComboBox change to update_coordinate_orientation
                ee_frame_combo.currentTextChanged.connect(
                    lambda _, s=f'robot_{i+1}': self.update_coordinate_orientation(s)
                )
                
                update_btn = QPushButton("Update EE Frame")
                update_btn.clicked.connect(lambda _, s=f'robot_{i+1}', robot_name = robot_name: self.update_ee_frame_and_ik(s, robot_name))
                
                # --- Layout: Combo and label in one row, button in next row ---
                ee_vlayout = QVBoxLayout()
                ee_hlayout = QHBoxLayout()
                ee_hlayout.addWidget(QLabel("EE Frame:"))
                ee_hlayout.addWidget(ee_frame_combo)
                ee_vlayout.addLayout(ee_hlayout)
                ee_vlayout.addWidget(update_btn)
                group_layout.addLayout(ee_vlayout)

                # Joint Pose group
                if 'joint_pose' in content:
                    jp = content['joint_pose']
                    sub_box = QGroupBox("Joint Angles")
                    sub_layout = QFormLayout()
                    angles_deg = jp.get('joint_angles_degrees', {})
                    import re
                    sorted_names = sorted(
                        angles_deg.keys(),
                        key=lambda name: int(re.search(r'\d+', name).group()) if re.search(r'\d+', name) else float('inf')
                    )
                    for name in sorted_names:
                        line = QLineEdit(f"")
                        self.fields[f"{f'robot_{i+1}'}.joint_pose.joint_angles_degrees.{name}"] = line
                        self.joint_fields.append(line)
                        line.editingFinished.connect(self.lock_cartesian_fields)
                        display_name = name.replace('_', ' ').title()
                        sub_layout.addRow(QLabel(f"{display_name} (°)"), line)
                    sub_box.setLayout(sub_layout)
                    group_layout.addWidget(sub_box)

                # Cartesian Pose group
                if 'cartesian_pose' in content:
                    cp = content['cartesian_pose']
                    sub_box = QGroupBox("Cartesian Pose")
                    sub_layout = QFormLayout()
                    for axis in ['x','y','z']:
                        line = QLineEdit(f"")
                        self.fields[f"{f'robot_{i+1}'}.cartesian_pose.ee_translation_m.{axis}"] = line
                        self.cartesian_fields.append(line)
                        line.editingFinished.connect(self.lock_joint_fields)
                        sub_layout.addRow(QLabel(f"{axis.upper()} (m)"), line)
                    for axis in ['roll','pitch','yaw']:
                        line = QLineEdit(f"")
                        self.fields[f"{f'robot_{i+1}'}.cartesian_pose.ee_rotation_euler_degrees.{axis}"] = line
                        self.cartesian_fields.append(line)
                        line.editingFinished.connect(self.lock_joint_fields)
                        sub_layout.addRow(QLabel(f"{axis.capitalize()} (°)"), line)
                    sub_box.setLayout(sub_layout)
                    group_layout.addWidget(sub_box)

                group_box.setLayout(group_layout)
                h_layout.addWidget(group_box)
                #i += 1

        self.update_fields_from_pose_data()
            

        # Add Buttons in the bottom
        button_layout = QHBoxLayout()
        get_current_btn = QPushButton("Fetch Values")
        get_current_btn.clicked.connect(self.update_pose_fields_callback)
        button_layout.addWidget(get_current_btn)

        move_to_btn = QPushButton("Move To Pose")
        move_to_btn.clicked.connect(self.move_to_callback)
        button_layout.addWidget(move_to_btn)

        display_target_btn = QPushButton("Display Target")
        display_target_btn.clicked.connect(self.display_target_callback)
        button_layout.addWidget(display_target_btn)

        save_btn = QPushButton("Save")
        save_btn.clicked.connect(self.save_or_rename_pose)
        button_layout.addWidget(save_btn)

        save_as_btn = QPushButton("Save As")
        save_as_btn.clicked.connect(
            lambda: self.save_or_rename_pose(show_file_selector=True)
        )
        button_layout.addWidget(save_as_btn)

        reset_btn = QPushButton("Reset")
        reset_btn.clicked.connect(self.reset_pose_fields_callback)
        button_layout.addWidget(reset_btn)

        self.layout.addLayout(h_layout)
        self.layout.addWidget(header_group)
        self.layout.addLayout(button_layout)

    def set_robot_fields_enabled(self, section, enabled):
        """Enable or disable all fields for a robot section."""
        for key, field in self.fields.items():
            if key.startswith(section + "."):
                field.setReadOnly(not enabled)
                if enabled:
                    field.setStyleSheet("")
                else:
                    field.setStyleSheet("background-color: #eee;")
    

    def lock_cartesian_fields(self):
        if self.locked_group is None:
            for field in self.cartesian_fields:
                field.setReadOnly(True)
                field.setStyleSheet("background-color: #eee;")
            self.locked_group = "joint"
        self.solve_fk = True
        self.solve_ik = False
        #logger.info("Locking cartesian fields, enabling FK solving")


    def lock_joint_fields(self):
        if self.locked_group is None:
            for field in self.joint_fields:
                field.setReadOnly(True)
                field.setStyleSheet("background-color: #eee;")
            self.locked_group = "cartesian"
        self.solve_ik = True
        self.solve_fk = False
        #logger.info("Locking joint fields, enabling IK solving")
    

    def unlock_fields(self):
        for idx, section in enumerate(self.pose_robot_names):
            if section in self.parent.get_robot_list():
                if self.robot_enable_checkboxes[section].isChecked():
                    for key, field in self.fields.items():
                        if key.startswith(f'robot_{idx+1}' + "."):
                            field.setReadOnly(False)
                            field.setStyleSheet("")
        self.locked_group = None
        self.solve_fk = False
        self.solve_ik = False
        #logger.info("Unlocking all fields")

    def display_target_callback(self):
        self.update_pose_data_from_fields()
        robot_checkbox_states = {}
        for robot_name, checked in self.robot_enable_checkboxes.items():
            robot_checkbox_states[robot_name] = checked.isChecked()
        display_target(self.node, self.pose_robot_names, self.parent, self.pose_data, robot_checkbox_states, self.traj_commander)
        self.update_fields_from_pose_data()

    def move_to_callback(self):

        if self.parent.get_servo_active():
            logger.info(f"Servoing running, shutting off first")
            self.parent.stop_servo()

        self.update_pose_data_from_fields()
        robot_checkbox_states = {}
        for robot_name, checked in self.robot_enable_checkboxes.items():
            robot_checkbox_states[robot_name] = checked.isChecked()
        move_to(self.node, self.pose_robot_names, self.parent, self.pose_data, robot_checkbox_states, self.traj_commander)
        self.update_fields_from_pose_data()
    
    def save_or_rename_pose(self, show_file_selector=True):
        self.update_pose_data_from_fields()

        new_name = self.pose_data['header'].get('pose_name', '').strip()
        if not new_name:
            QMessageBox.critical(self, "Error", "Pose name cannot be empty.")
            return

        old_path = getattr(self, "file_path", "") or ""
        old_base = os.path.splitext(os.path.basename(old_path))[0] if old_path else ""
        dir_path = os.path.dirname(old_path) if old_path else self.parent.get_save_dir()

        def finalize_save(path: str, info_msg: str):
            self.file_path = path
            write_yaml(self.pose_data, self.file_path)
            self.initial_pose_data = copy.deepcopy(self.pose_data)
            QMessageBox.information(self, new_name, info_msg)
            self.update_fields_from_pose_data()

        if show_file_selector:
            target_path, _ = QFileDialog.getSaveFileName(
                self, "Save Pose As", dir_path, "YAML Files (*.yaml *.yml)"
            )
            if not target_path:
                return
            # update name from chosen file
            new_name = os.path.splitext(os.path.basename(target_path))[0]
            self.pose_data['header']['pose_name'] = new_name
            finalize_save(target_path, f"Pose saved as '{new_name}'.")
            # emit rename if base changed
            #if old_base and new_name != old_base:
            #    self.pose_renamed.emit(old_base, new_name)
            return

        # no file selector: save/rename in place
        if not self.file_path:
            # no existing path, create one in directory
            self.file_path = os.path.join(dir_path, f"{new_name}.yaml")
            old_path = self.file_path  # so rename logic below treats as same
            old_base = new_name

        if new_name != old_base:
            new_filename = new_name + ".yaml"
            new_path = os.path.join(dir_path, new_filename)
            # ask for overwrite if target exists and is different from current
            if os.path.exists(new_path) and new_path != old_path:
                resp = QMessageBox.question(
                    self,
                    "Overwrite File",
                    f"A file named '{new_filename}' already exists. Overwrite?",
                    QMessageBox.Yes | QMessageBox.No
                )
                if resp != QMessageBox.Yes:
                    return
            finalize_save(new_path, f"Pose saved and renamed to '{new_filename}'.")
            # remove old file if different
            if old_path and os.path.exists(old_path) and old_path != self.file_path:
                try:
                    os.remove(old_path)
                except Exception as e:
                    logger.warning(f"Could not remove old file: {e}")
            if old_base != new_name:
                self.pose_renamed.emit(old_base, new_name)
        else:
            # name unchanged: overwrite existing
            finalize_save(self.file_path, "Pose updated.")

    

    def update_pose_data_from_fields(self):
        for key, field in self.fields.items():
            keys = key.split('.')
            ref = self.pose_data
            for k in keys[:-1]:
                ref = ref[k]
            try:
                ref[keys[-1]] = yaml.safe_load(field.text())
            except yaml.YAMLError:
                ref[keys[-1]] = field.text()

        self.pose_data['header']['last_edited'] = datetime.now().strftime("%d.%m.%Y %H:%M:%S")
        self.pose_data['header']['pose_name'] = self.pose_name_edit.text().strip()
        self.pose_data['header']['description'] = self.pose_desc_edit.text().strip()

        for section in self.pose_data:
            if 'joint_pose' in self.pose_data[section]:
                if self.pose_data['header'][section] in self.parent.get_robot_list():
                    robot_name = self.pose_data['header'][section]
                    joint_angles_deg = self.pose_data[section]['joint_pose']['joint_angles_degrees']
                    angles_deg = list(joint_angles_deg.values())

                    joint_angles_rad = [round(deg * pi / 180, 6) for deg in angles_deg]
                    joint_pose = self.node._get_joint_state(robot_name)
                    joint_names = joint_pose['joint_names']
                    base_frame = self.pose_data[section]['cartesian_pose']['base_frame_name']
                    ee_frame = self.pose_data[section]['cartesian_pose']['ee_frame_name']
                    base_frame_full = f'{robot_name}/{base_frame}'
                    ee_frame_full = f'{robot_name}/{ee_frame}'
                    self.node.robot_config[robot_name]['base_frame'] = base_frame_full
                    self.node.robot_config[robot_name]['ee_frame'] = ee_frame_full
                    if self.solve_fk:
                        update_fk(self.fk_client, joint_angles_rad, section, joint_names, base_frame_full, ee_frame_full, handle_fk_result,  self.pose_data, self.fk_pose_received)
                    elif self.solve_ik:
                        update_ik(self.ik_client, self.pose_data, section, robot_name, base_frame_full, ee_frame_full, joint_angles_rad, joint_names, handle_ik_result, self.file_path)
                    
                    sleep(1) #To ensure the FK/IK services have time to process before publishing
                    self.update_pose_fields(section)


    def update_fields_from_pose_data(self):
        # Header fields
        header = self.pose_data.get('header', {})
        self.pose_name_edit.setText(header.get('pose_name', ''))
        self.pose_desc_edit.setText(header.get('description', ''))
        # Other fields:
        for key, field in self.fields.items():
            keys = key.split('.')
            ref = self.pose_data
            for k in keys[:-1]:
                ref = ref.get(k, {})
            val = ref.get(keys[-1], '')
            # Numeric formatting
            if isinstance(field, QLineEdit):
                if isinstance(val, (int, float)):
                    # Determine rounding
                    if 'ee_translation_m' in keys:
                        v = round(float(val), 4)
                    else:
                        v = round(float(val), 1)
                    # Replace -0.0 with 0.0
                    if abs(v) == 0.0:
                        v = 0.0
                    text = f"{v}"
                else:
                    text = str(val)
                field.setText(text)
        # Unlock all fields
        self.unlock_fields()
    

    def reset_pose_fields_callback(self):
        resp = QMessageBox.question(
            self,
            "Reset Fields",
            "Restore fields to initial values? Unsaved changes will be lost.",
            QMessageBox.Yes | QMessageBox.No
        )
        if resp != QMessageBox.Yes:
            return
        # If yes, proceed with reset
        self.pose_data = copy.deepcopy(self.initial_pose_data)
        self.update_fields_from_pose_data()


    # --- Replace the body of update_pose_fields_callback with a yes/no dialog:
    def update_pose_fields_callback(self):
        resp = QMessageBox.question(
            self,
            "Fetch Values",
            "Overwrite fields with current robot pose?",
            QMessageBox.Yes | QMessageBox.No
        )
        if resp != QMessageBox.Yes:
            return
        # If yes, perform lookup and update
        self.update_pose_data_from_fields()
        header = self.pose_data["header"] 
        which_robot = 0
        for i, robot in enumerate(self.parent.get_robot_list()):
            if robot in self.pose_robot_names or (len(self.parent.get_robot_list()) == 1 and len(self.pose_robot_names) == 1):
                which_robot |= (1 << i)
            if (len(self.parent.get_robot_list()) == 1 and len(self.pose_robot_names) == 1):
                self.node.robot_config[robot]['base_frame'] = self.parent.get_base_frame(0)
                self.node.robot_config[robot]['ee_frame'] = self.parent.get_ee_frame(0)
        self.pose_data = self.node.lookup_pose(which_robot)
        self.pose_data["header"] = header 
        self.update_fields_from_pose_data()
    

    def update_cartesian_pose_fields(self, robot_state, section, base_frame, ee_frame):
        if robot_state is None:
            QMessageBox.critical(self, "FK Error", "Failed to compute FK for updated joints.")
            return
        logger.info(f"FK result for {section}: {robot_state}")
        o = robot_state.orientation
        roll, pitch, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
        p = robot_state.position
        # Update pose_data
        self.pose_data[section]['cartesian_pose'] = {
            'base_frame_name': base_frame,
            'ee_frame_name': ee_frame,
            'ee_translation_m': {
                'x': round(p.x, 4),
                'y': round(p.y, 4),
                'z': round(p.z, 4)
            },
            'ee_rotation_euler_degrees': {
                'roll': round(rad_to_deg(roll), 1),
                'pitch': round(rad_to_deg(pitch), 1),
                'yaw': round(rad_to_deg(yaw), 1)
            }
        }
        self.update_pose_fields(section)
    def update_pose_fields(self, section):
        # Update UI fields
        for axis in ['x','y','z']:
            key = f"{section}.cartesian_pose.ee_translation_m.{axis}"
            if key in self.fields:
                v = self.pose_data[section]['cartesian_pose']['ee_translation_m'][axis]
                self.fields[key].setText(f"{round(v, 4)}")
        for axis in ['roll','pitch','yaw']:
            key = f"{section}.cartesian_pose.ee_rotation_euler_degrees.{axis}"
            if key in self.fields:
                v = self.pose_data[section]['cartesian_pose']['ee_rotation_euler_degrees'][axis]
                self.fields[key].setText(f"{round(v, 1)}")

        for joints in self.pose_data[section]['joint_pose']['joint_angles_degrees']:

            key = f"{section}.joint_pose.joint_angles_degrees.{joints}"
            if key in self.fields:
                v = self.pose_data[section]['joint_pose']['joint_angles_degrees'][joints]
                self.fields[key].setText(f"{round(v, 1)}")

    def update_ee_frame_and_ik(self, section, robot_name):
        # Get selected display EE frame
        display_ee_frame = self.ee_frame_combos[section].currentText()
        # Convert display name to full frame name
        full_ee_frame = self.ee_frame_display_to_full[section][display_ee_frame]
        # Update pose_data
        self.pose_data[section]['cartesian_pose']['ee_frame_name'] = display_ee_frame

        # Prepare IK call
        base_frame = self.pose_data[section]['cartesian_pose']['base_frame_name']
        full_base_frame = self.ee_frame_display_to_full[section][base_frame]
        joint_pose = self.pose_data[section]['joint_pose']
        angles_deg = joint_pose['joint_angles_degrees'].values()
        # Convert angles to radians
        angles_rad = [deg_to_rad(angle) for angle in angles_deg]
        joint_angles_rad = angles_rad

        joint_pose = self.node._get_joint_state(robot_name)
        joint_names = joint_pose['joint_names']

        # Call update_ik
        update_ik(
            self.ik_client,
            self.pose_data,
            section,
            robot_name,
            full_base_frame,
            full_ee_frame,
            joint_angles_rad,
            joint_names,
            handle_ik_result,
            self.file_path
        )
        # Optionally, update UI after a short delay
        sleep(1)
        
        self.update_fields_from_pose_data()

    def update_coordinate_orientation(self, section):
        # 1) What *was* our EE frame?  (this is your "current" frame)
        current_frame = self.pose_data[section]['cartesian_pose']['ee_frame_name']
        current_frame_full = self.ee_frame_display_to_full[section][current_frame]

        display_ee_frame = self.ee_frame_combos[section].currentText()
        target_frame = self.ee_frame_display_to_full[section][display_ee_frame]
        T = self.node._get_transformation_matrix(current_frame_full, target_frame)
        tf_matrix = np.array(T['tf_matrix'])        # shape (4,4)
        q_tf = quaternion_from_matrix(tf_matrix)

        # Prepare to collect the new euler angles and build the final orientation quaternion
        new_orient = {}
        r = deg_to_rad(float(self.fields[f"{section}.cartesian_pose.ee_rotation_euler_degrees.roll"].text()))
        p = deg_to_rad(float(self.fields[f"{section}.cartesian_pose.ee_rotation_euler_degrees.pitch"].text()))
        y = deg_to_rad(float(self.fields[f"{section}.cartesian_pose.ee_rotation_euler_degrees.yaw"].text()))

        # 3) Build a *single* target quaternion from that triple
        q_target = quaternion_from_euler(r, p, y, axes='sxyz')
        # 4) Compose it with your frame transform
        q_new = quaternion_multiply(q_target, q_tf )


        r, p, y = euler_from_quaternion(q_new, axes='sxyz')
        new_orient['roll'] = np.rad2deg({'roll': r, 'pitch': p, 'yaw': y}['roll'])
        new_orient['pitch'] = np.rad2deg({'roll': r, 'pitch': p, 'yaw': y}['pitch'])
        new_orient['yaw'] = np.rad2deg({'roll': r, 'pitch': p, 'yaw': y}['yaw'])


        # Write them back into your UI fields
        for axis, val in new_orient.items():
            key = f"{section}.cartesian_pose.ee_rotation_euler_degrees.{axis}"
            self.fields[key].setText(f"{val:.3f}")

        # --- Build geometry_msgs.msg.Pose ---
        robot_state = Pose()
        # Translation
        for axis in ['x', 'y', 'z']:
            key = f"{section}.cartesian_pose.ee_translation_m.{axis}"
            setattr(robot_state.position, axis, float(self.fields[key].text()))
        # Orientation (use the last computed q_new)
        robot_state.orientation.x = q_new[0]
        robot_state.orientation.y = q_new[1]
        robot_state.orientation.z = q_new[2]
        robot_state.orientation.w = q_new[3]

        # Get base_frame and call update_cartesian_pose_fields
        base_frame = self.pose_data[section]['cartesian_pose']['base_frame_name']
        self.update_cartesian_pose_fields(robot_state, section, base_frame, display_ee_frame)

