import subprocess
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QFormLayout, QLineEdit, QSpinBox, QComboBox, QSpacerItem, QSizePolicy,
    QCheckBox, QPushButton, QMessageBox, QHBoxLayout, QSlider, QGroupBox, QFrame, QFileDialog
)
from PySide6.QtCore import Qt
import os
import yaml
import rclpy
import subprocess
from PySide6.QtCore import QTimer
from controller_manager_msgs.srv import SwitchController
from moveit_msgs.srv import ServoCommandType
from pose_recorder.utils import set_node_parameters, get_node_parameters, get_tf_frames, get_node_names


logger = rclpy.logging.get_logger("SpaceMouseTab")

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QFormLayout, QRadioButton,
    QLabel, QComboBox, QSlider, QLineEdit, QPushButton, QCheckBox, QButtonGroup
)
from PySide6.QtCore import Qt, Signal
import rclpy
from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.task import Future
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import Parameter, ParameterValue


class SpaceMouseTab(QWidget):
    """
    Tab for configuring and controlling a SpaceMouse device.
    Publishes TwistStamped to selected /neura_X/cmd_vel topics.
    """

    def __init__(self, parent, node, executor):
        super().__init__(parent)
        self.node = node
        self.parent = parent
        self.frame_cbs = {}

        self.executor = executor
        self.servo_proc = None
        self._svc_clients = {}
        self._cmd_type_clients = {}
        self.servoing_active = False
        self.config_path = None

        self.axis_enable_checkboxes = {}

        self.create_layout()
    
    def create_layout(self):
        layout = QVBoxLayout(self)

        self.teleop_param_edits = {}

        # outer group
        params_grp = QGroupBox("Teleop Twist Joy")
        params_grp.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        layout.addWidget(params_grp)
        form = QFormLayout(params_grp)
        form.setContentsMargins(8, 8, 8, 8)
        form.setSpacing(6)

        # --- Config Display ---
        self.config_label = QLabel("<none>")
        font = self.config_label.font()
        font.setBold(True)
        self.config_label.setFont(font)
        form.addRow("Active Config:", self.config_label)

        # --- Teleop node selector ---
        self.node_cb = QComboBox()
        self._populate_teleop_nodes()
        form.addRow("Teleop Node:", self.node_cb)
        #self.node_cb.currentTextChanged.connect(self._fetch_teleop_params)

        # Controlled TF Frame
        self.tf_frame_cb = QComboBox()
        self._populate_tf_frames()
        form.addRow("Control Frame:", self.tf_frame_cb)
        self.teleop_param_edits["frame"] = self.tf_frame_cb

        blank = QSpacerItem(0, 8, QSizePolicy.Minimum, QSizePolicy.Fixed)
        form.addItem(blank)

        # Translation fields
        tlay = QHBoxLayout()
        self.teleop_param_edits["axis_linear"] = {}
        for axis in ("x","y","z"):
            line = QLineEdit("-")
            tlay.addWidget(QLabel(f"{axis.upper()}:"))
            tlay.addWidget(line)
            self.teleop_param_edits["axis_linear"][axis]=line
        form.addRow("Linear Axes:", tlay)

        tlay = QHBoxLayout()
        self.teleop_param_edits["scale_linear"] = {}
        for axis in ("x","y","z"):
            line = QLineEdit("-")
            tlay.addWidget(QLabel(f"{axis.upper()}:"))
            tlay.addWidget(line)
            self.teleop_param_edits["scale_linear"][axis]=line
        form.addRow("Linear Scale:", tlay)

        tlay = QHBoxLayout()
        self.teleop_param_edits["scale_linear_turbo"] = {}
        for axis in ("x","y","z"):
            line = QLineEdit("-")
            tlay.addWidget(QLabel(f"{axis.upper()}:"))
            tlay.addWidget(line)
            self.teleop_param_edits["scale_linear_turbo"][axis]=line
        form.addRow("Linear Turbo:", tlay)

        # Axis Enable Checkboxes
        axis_en_layout = QHBoxLayout()
        for axis in ["X", "Y", "Z"]:
            cb = QCheckBox()
            cb.setChecked(True)
            cb.stateChanged.connect(lambda _, a=axis.lower(): self._on_teleop_axis_enable_changed(a))
            axis_en_layout.addWidget(cb)
            self.axis_enable_checkboxes[axis.lower()] = cb
        form.addRow("Enabled:", axis_en_layout)

        blank = QSpacerItem(0, 8, QSizePolicy.Minimum, QSizePolicy.Fixed)
        form.addItem(blank)

        # Rotation fields
        tlay = QHBoxLayout()
        self.teleop_param_edits["axis_angular"] = {}
        for axis,label in (("roll","R"),("pitch","P"),("yaw","Y")):
            line = QLineEdit("-")
            tlay.addWidget(QLabel(f"{label}:"))
            tlay.addWidget(line)
            self.teleop_param_edits["axis_angular"][axis]=line
        form.addRow("Angular Axes:", tlay)

        # Rotation fields
        tlay = QHBoxLayout()
        self.teleop_param_edits["scale_angular"] = {}
        for axis,label in (("roll","R"),("pitch","P"),("yaw","Y")):
            line = QLineEdit("-")
            tlay.addWidget(QLabel(f"{label}:"))
            tlay.addWidget(line)
            self.teleop_param_edits["scale_angular"][axis]=line
        form.addRow("Angular Scale:", tlay)

        tlay = QHBoxLayout()
        self.teleop_param_edits["scale_angular_turbo"] = {}
        for axis,label in (("roll","R"),("pitch","P"),("yaw","Y")):
            line = QLineEdit("-")
            tlay.addWidget(QLabel(f"{label}:"))
            tlay.addWidget(line)
            self.teleop_param_edits["scale_angular_turbo"][axis]=line
        form.addRow("Angular Turbo:", tlay)

        # Axis Enable Checkboxes
        axis_en_layout = QHBoxLayout()
        for axis in ["Roll", "Pitch", "Yaw"]:
            cb = QCheckBox()
            cb.setChecked(True)
            cb.stateChanged.connect(lambda _, a=axis.lower(): self._on_teleop_axis_enable_changed(a))
            axis_en_layout.addWidget(cb)
            self.axis_enable_checkboxes[axis.lower()] = cb
        form.addRow("Enabled:", axis_en_layout)


        blank = QSpacerItem(0, 8, QSizePolicy.Minimum, QSizePolicy.Fixed)
        form.addItem(blank)

        # # --- require_enable_button & indices ---
        vlay = QVBoxLayout()

        require_enable_button_cb = QCheckBox("Require Enable Button")
        require_enable_button_cb.setChecked(True)
        vlay.addWidget(require_enable_button_cb)
        self.teleop_param_edits["require_enable_button"] = require_enable_button_cb

        hlay = QHBoxLayout()
        hlay.addWidget(QLabel("Enable Button:   "))

        enable_button_cb = QComboBox()
        enable_button_cb.addItems(["0", "1", "-1"])
        hlay.addWidget(enable_button_cb)
        self.teleop_param_edits["enable_button"] = enable_button_cb
        hlay.addStretch()

        vlay.addItem(hlay)

        hlay = QHBoxLayout()
        hlay.addWidget(QLabel("Turbo Button:     "))

        enable_turbo_button_cb = QComboBox()
        enable_turbo_button_cb.addItems(["0", "1", "-1"])
        self.teleop_param_edits["enable_turbo_button"] = enable_turbo_button_cb
        hlay.addWidget(enable_turbo_button_cb)
        hlay.addStretch()

        vlay.addItem(hlay)

        form.addRow("User Buttons: ", vlay)


        # --- Config save, load, fetch and apply buttons ---
        btn_row = QWidget()
        btn_layout = QHBoxLayout(btn_row)
        
        self.refresh_nodes_btn = QPushButton("Refresh")
        self.refresh_nodes_btn.clicked.connect(self._on_refresh)
        btn_layout.addWidget(self.refresh_nodes_btn)

        self.fetch_params_btn = QPushButton("Fetch")
        self.fetch_params_btn.clicked.connect(self._fetch_teleop_params)
        btn_layout.addWidget(self.fetch_params_btn)

        self.apply_params_btn = QPushButton("Apply")
        self.apply_params_btn.clicked.connect(self._apply_teleop_params)
        btn_layout.addWidget(self.apply_params_btn)
        
        self.load_file_btn = QPushButton("Load")
        self.load_file_btn.clicked.connect(self._load_teleop_params_from_file)
        btn_layout.addWidget(self.load_file_btn)

        self.save_btn = QPushButton("Save")
        self.save_btn.clicked.connect(
            lambda: self._save_teleop_params_to_file(show_file_selector=False)
        )
        btn_layout.addWidget(self.save_btn)

        self.save_as_btn = QPushButton("Save As")
        self.save_as_btn.clicked.connect(
            lambda: self._save_teleop_params_to_file(show_file_selector=True)
        )
        btn_layout.addWidget(self.save_as_btn)

        layout.addWidget(btn_row)
        layout.addStretch()


    def _populate_tf_frames(self):
        """
        Fill the given QComboBox with all available TF frames for the current robot.
        """
        combo = self.tf_frame_cb
        robots = self.parent.get_robot_list()

        tf_whitelist, tf_override_keywords = self.parent.get_tf_filter_params()
        frames = get_tf_frames(
                self.node,
                namespaces=robots,
                whitelist=tf_whitelist,
                override_keywords=tf_override_keywords,
            )
        prev = combo.currentText()
        combo.clear()
        combo.addItems(frames)
        if prev:
            idx = combo.findText(prev)
            if idx >= 0:
                combo.setCurrentIndex(idx)
            elif combo.count():
                combo.setCurrentIndex(0)

    
    def _populate_teleop_nodes(self):
        """
        Fill the given QComboBox with all running teleop_twist_joy nodes
        in the form '<namespace>/<node_name>'.
        """
        combo = self.node_cb
        combo.clear()

        try:
            entries = get_node_names(self.node, 'teleop_twist_joy', ignore_case=True)
            entries.sort()
            combo.addItems(entries)
        except Exception as e:
            logger.error(f"Failed to list teleop nodes: {e}")
    

    def _on_refresh(self):

        logger.info(f"Refreshing...")
        self._populate_teleop_nodes()
        self._populate_tf_frames()
        return
    
    def _fetch_teleop_params(self):

        node_text = self.node_cb.currentText().strip()
        if not node_text:
            self._populate_teleop_nodes()
            node_text = self.node_cb.currentText().strip()
            if not node_text:
                QMessageBox.information(self, "Parameters", "No teleop_twist_joy node available.")
                return
        
        node_name = self.node_cb.currentText().lstrip('/')

        resp = QMessageBox.question(
            self,
            "Fetch Teleop Params",
            f"Overwrite Fields with Parameters of\n\n{node_name}?",
            QMessageBox.Yes | QMessageBox.No
        )
        if resp != QMessageBox.Yes:
            return

        params = ["require_enable_button", "enable_button", "enable_turbo_button", "frame"]

        # Dynamically append parameter names depending on enabled axes
        for group in ["axis_linear", "scale_linear", "scale_linear_turbo",
                    "axis_angular", "scale_angular", "scale_angular_turbo"]:
            for axis in ("x", "y", "z") if "linear" in group else ("roll", "pitch", "yaw"):
                # Skip scale_* values if the axis checkbox is unchecked
                if group.startswith("scale_"):
                    checkbox = self.axis_enable_checkboxes.get(axis.lower())
                    if checkbox and not checkbox.isChecked():
                        continue
                params.append(f"{group}.{axis}")

        try:
            result = get_node_parameters(node_name, params, 5.0)
        except Exception as e:
            logger.error(f"Param call failed: {e}")
            return
            
        if result:
            # Read out button states and update button checkboxes/dropdowns
            self.teleop_param_edits["require_enable_button"].setChecked(
                bool(result["require_enable_button"])
            )
            self.teleop_param_edits["enable_button"].setCurrentText(
                str(result["enable_button"])
            )
            self.teleop_param_edits["enable_turbo_button"].setCurrentText(
                str(result["enable_turbo_button"])
            )

            
            # Read out frame ID and write to dropdown
            frame_value = str(result["frame"])
            index = self.tf_frame_cb.findText(frame_value)
            if index >= 0:
                self.tf_frame_cb.setCurrentIndex(index)
            else:
                self.tf_frame_cb.addItem(frame_value)
                self.tf_frame_cb.setCurrentIndex(self.tf_frame_cb.count() - 1)

            ax_names = params[4:]

            # Read out parameter values (axis directions, speeds) and write them into text fields
            for name in ax_names:
                # split "group.axis"
                value = result[name] 
                group, axis = name.split('.', 1)
                widget = self.teleop_param_edits.get(group, {}).get(axis)
                if widget:
                    widget.setText(str(value))
            logger.info("Teleop parameters loaded")
        else:
            logger.info("Connection error: Could not fetch teleop parameters.")


    def _apply_teleop_params(self):

        node_text = self.node_cb.currentText().strip()
        if not node_text:
            QMessageBox.information(self, "Parameters", "No teleop_twist_joy node available.")
            return
        
        node_name = self.node_cb.currentText().lstrip('/')
        srv_name = f"/{node_name}/set_parameters"
        client: Client = self.node.create_client(SetParameters, srv_name)

        if not client.wait_for_service(timeout_sec=1.0):
            QMessageBox.warning(self, "Parameters", f"Could not contact {srv_name}")
            return
        
        # Apply control_utils method and use utils function
        params_dict = {
            "frame":                    str(self.tf_frame_cb.currentText()),
            "require_enable_button":    bool(self.teleop_param_edits["require_enable_button"].isChecked()),
            "enable_button":            int(self.teleop_param_edits["enable_button"].currentText()),
            "enable_turbo_button":      int(self.teleop_param_edits["enable_turbo_button"].currentText()),
        } 

        for group, axes in self.teleop_param_edits.items():
            if not (group.startswith("axis_") or group.startswith("scale_")):
                continue
            for axis, line_edit in axes.items():
                full_name = f"{group}.{axis}"
                text = line_edit.text().strip()

                # Skip if input is empty
                if not text or text == "-":
                    continue

                # Try parsing as int, then float
                try:
                    if group.startswith("axis_"):
                        value = int(text)
                    else:
                        # Suppress scale if axis is unchecked / Disable axis temporarily
                        checkbox = self.axis_enable_checkboxes.get(axis.lower())
                        if checkbox and not checkbox.isChecked():
                            value = 0.0
                        else:
                            value = float(text)
                except ValueError:
                    logger.warning(f"Invalid input for {full_name}: {text}")
                    continue

                params_dict[full_name] = value

        set_node_parameters(node_name, params_dict, self.node)


    def _on_teleop_axis_enable_changed(self, axis_name):
        axis_map = {
            "x": ("axis_linear", "scale_linear", "scale_linear_turbo"),
            "y": ("axis_linear", "scale_linear", "scale_linear_turbo"),
            "z": ("axis_linear", "scale_linear", "scale_linear_turbo"),
            "roll": ("axis_angular", "scale_angular", "scale_angular_turbo"),
            "pitch": ("axis_angular", "scale_angular", "scale_angular_turbo"),
            "yaw": ("axis_angular", "scale_angular", "scale_angular_turbo"),
        }

        enabled = self.axis_enable_checkboxes[axis_name].isChecked()
        for group in axis_map.get(axis_name, []):
            if axis_name in self.teleop_param_edits.get(group, {}):
                self.teleop_param_edits[group][axis_name].setEnabled(enabled)


    def _load_teleop_params_from_file(self):
        dir_path = self.parent.get_config_dir()
        
        """Open a YAML, read under ros__parameters, populate the fields."""
        self.config_path, _ = QFileDialog.getOpenFileName(
            self,
            "Load Teleop Config",
             dir_path,
             "YAML Files (*.yaml *.yml)"
        )
        if not self.config_path:
            return
        try:
            
            # Update config text display
            name = os.path.basename(self.config_path)
            stem, _ = os.path.splitext(name)
            self.config_label.setText(stem)
            
            with open(self.config_path, 'r') as f:
                cfg = yaml.safe_load(f)
            params = cfg.get('ros__parameters', {})
            # for each group/axis in our edits, set value if present
            for group, axes in self.teleop_param_edits.items():
                if not (group.startswith("axis_") or group.startswith("scale_")):
                    continue
                sub = params.get(group, {})
                for axis, widget in axes.items():
                    val = sub.get(axis)
                    if val is not None:
                        widget.setText(str(val))
            
            # restore the enable‐button settings
            if 'require_enable_button' in params:
                self.teleop_param_edits["require_enable_button"].setChecked(
                    bool(params['require_enable_button'])
                )
            if 'enable_button' in params:
                self.teleop_param_edits["enable_button"].setCurrentText(
                    str(params['enable_button'])
                )
            if 'enable_turbo_button' in params:
                self.teleop_param_edits["enable_turbo_button"].setCurrentText(
                    str(params['enable_turbo_button'])
                )
            if 'frame' in params:
                self.teleop_param_edits["frame"].setCurrentText(
                    str(params['frame'])
                )

            QMessageBox.information(self, "Load Parameters", f"Loaded from {os.path.basename(self.config_path)}")
        except Exception as e:
            QMessageBox.critical(self, "Load Failed", str(e))


    def _save_teleop_params_to_file(self, show_file_selector=True):

        logger.info(f"Save config at: {self.config_path}")
        
        if show_file_selector:
            dir_path = self.parent.get_config_dir()

            """Gather current fields and write out a YAML template under ros__parameters."""
            self.config_path, _ = QFileDialog.getSaveFileName(self, "Save Teleop Config", dir_path, "YAML Files (*.yaml *.yml)")
        if not self.config_path:
            logger.info(f"No config path found!")
            return
        # ensure .yaml
        base, _ = os.path.splitext(self.config_path)
        self.config_path = base + '.yaml'

        # Update config text display
        name = os.path.basename(self.config_path)
        stem, _ = os.path.splitext(name)
        self.config_label.setText(stem)
        
        out = {'ros__parameters': {}}
        for group, axes in self.teleop_param_edits.items():
            if not (group.startswith("axis_") or group.startswith("scale_")):
                continue
            out['ros__parameters'][group] = {}
            for axis, widget in axes.items():
                text = widget.text().strip()
                try:
                    # preserve numeric types
                    if text:
                        if '.' in text:
                            out['ros__parameters'][group][axis] = float(text)
                        else:
                            out['ros__parameters'][group][axis] = int(text)
                except ValueError:
                    out['ros__parameters'][group][axis] = text
        
        out['ros__parameters']['require_enable_button'] = self.teleop_param_edits["require_enable_button"].isChecked()
        out['ros__parameters']['enable_button'] = int(self.teleop_param_edits["enable_button"].currentText())
        out['ros__parameters']['enable_turbo_button'] = int(self.teleop_param_edits["enable_turbo_button"].currentText())
        out['ros__parameters']['frame'] = self.teleop_param_edits["frame"].currentText()
        
        try:
            with open(self.config_path, 'w') as f:
                yaml.safe_dump(out, f)
            QMessageBox.information(self, "Save Teleop Config", f"Saved to {os.path.basename(self.config_path)}")
        except Exception as e:
            QMessageBox.critical(self, "Save Failed", str(e))
