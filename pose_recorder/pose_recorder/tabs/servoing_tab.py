import signal
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QFormLayout, QLineEdit, QSpinBox, QComboBox, QSpacerItem, QSizePolicy,
    QCheckBox, QPushButton, QMessageBox, QHBoxLayout, QSlider, QGroupBox, QFrame, QFileDialog
)
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QDoubleSpinBox

import os
import yaml
import rclpy
import subprocess
from PySide6.QtCore import QTimer
from controller_manager_msgs.srv import SwitchController
from moveit_msgs.srv import ServoCommandType
from pose_recorder.utils import set_node_parameters, get_node_parameters, get_node_names, kill_nodes_by_keyword
from PySide6.QtCore import QProcess, Signal
from moveit_msgs.msg import ServoStatus
from pose_recorder.tabs.geo_constraints_tab import ControllerGUI



logger = rclpy.logging.get_logger("ServoingTab")

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
from typing import Any



class ServoingTab(QWidget):

    """
    Tab for configuring the Moveit Servo nodes.
    """
    # Signals to notify MainWindow to start/stop servo on a given robot
    servo_start = Signal(str)       # robot_name
    servo_stop  = Signal(str)       # robot_name
    servo_mute  = Signal(str, bool) # robot_name, mute_on

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
        self._servoing_active = False  # GUI-level servoing state for Start/Stop gating
        # __init__ (add this attribute)
        self._servo_nodes: list[str] = []
        self._original_scales_by_node: dict[str, dict] = {}
        self._all_nodes_text = "All Nodes"


        robots = self.parent.get_robot_list()
        # track pause/unpause state per servo node
        self._paused_states: dict[str, bool] = {}

        self.axis_enable_checkboxes = {}

        self._status_sub = None

        # pre-create service clients for all robots available
        for robot_name in robots:
            svc = self.node.create_client(
                SwitchController,
                f"/{robot_name}/controller_manager/switch_controller"
            )
            cli = self.node.create_client(
                ServoCommandType,
                f"/{robot_name}/servo_node/switch_command_type"
            )
            self._svc_clients[robot_name] = svc
            self._cmd_type_clients[robot_name] = cli
        
        self._param_cb_group = ReentrantCallbackGroup()
        self._param_clients = {}  # cache SetParameters clients per node
        # track launched servo launch processes (could be one or two for "combined")
        self._servo_launch_procs: list[subprocess.Popen] = []
        # controller node process handle
        self._controller_proc: subprocess.Popen | None = None
        self._controller_poll_timer = QTimer(self)
        self._controller_poll_timer.setInterval(500)
        self._controller_poll_timer.timeout.connect(self._poll_controller_proc)

        # track when we've seen a full RobotState
        self._state_ready = False
        self._state_sub = None
        self._wait_timer = QTimer(self)
        self._wait_timer.setInterval(200)
        self._wait_timer.timeout.connect(self._attempt_switch_when_ready)

        # just in case some old servo nodes are still running, kill them
        self.kill_all_servo_nodes()
        self.create_layout()
    
    def create_layout(self):
        layout = QVBoxLayout(self)

        # MoveIt Servo parameters
        servo_grp = QGroupBox("MoveIt Servo Parameters")
        servo_grp.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        layout.addWidget(servo_grp)
        form = QFormLayout(servo_grp)
        form.setContentsMargins(8, 8, 8, 8)
        form.setSpacing(6)

        # 1) Servo node selector
        self.servo_node_cb = QComboBox()
        form.addRow("Servo Node:", self.servo_node_cb)
        self.servo_node_cb.currentTextChanged.connect(self._fetch_servo_params)
        self.servo_node_cb.currentTextChanged.connect(self._update_pause_button_state)
        #self.servo_node_cb.currentTextChanged.connect(self._on_servo_node_changed)


        blank = QSpacerItem(0, 8, QSizePolicy.Minimum, QSizePolicy.Fixed)
        form.addItem(blank)

        # 4) apply_twist_commands_about_ee_frame checkbox
        self.apply_about_ee_cb = QCheckBox("Apply Cartesian Twist about EE Frame")
        form.addRow("Cartesian Twist", self.apply_about_ee_cb)

        # 5) scale.linear & scale.rotational
        self.scale_linear_sb       = QDoubleSpinBox()
        self.scale_linear_sb.setRange(0, 25)
        self.scale_linear_sb.setDecimals(1)
        self.scale_linear_sb.setSingleStep(0.1)

        self.scale_rotational_sb   = QDoubleSpinBox()
        self.scale_rotational_sb.setRange(0, 25)
        self.scale_rotational_sb.setDecimals(1)
        self.scale_rotational_sb.setSingleStep(0.1)

        form.addRow("Scale Linear [m/s]:",          self.scale_linear_sb)
        form.addRow("Scale Rotational [rad/s]:",    self.scale_rotational_sb)

        # 10) collisions & thresholds
        self.check_collisions_cb        = QCheckBox()
        self.check_collisions_cb.toggled.connect(self._on_check_collisions_toggled)
        self.collision_rate_sb          = QDoubleSpinBox()
        self.collision_rate_sb.setRange(0,100)
        self.collision_rate_sb.setDecimals(0)
        self.collision_rate_sb.setSingleStep(1)

        self.self_collision_thr_sb      = QDoubleSpinBox()
        self.self_collision_thr_sb.setRange(0,1)
        self.self_collision_thr_sb.setDecimals(3)
        self.self_collision_thr_sb.setSingleStep(0.01)

        self.scene_collision_thr_sb     = QDoubleSpinBox()
        self.scene_collision_thr_sb.setRange(0,1)
        self.scene_collision_thr_sb.setDecimals(3)
        self.scene_collision_thr_sb.setSingleStep(0.01)

        self.scene_collision_thr_sb     = QDoubleSpinBox()
        self.scene_collision_thr_sb.setRange(0,1)
        self.scene_collision_thr_sb.setDecimals(3)
        self.scene_collision_thr_sb.setSingleStep(0.01)

        self.scene_collision_stop_sb     = QDoubleSpinBox()
        self.scene_collision_stop_sb.setRange(0,1)
        self.scene_collision_stop_sb.setDecimals(3)
        self.scene_collision_stop_sb.setSingleStep(0.01)

        self.prediction_steps_sb     = QDoubleSpinBox()
        self.prediction_steps_sb.setRange(0,10)
        self.prediction_steps_sb.setDecimals(0)
        self.prediction_steps_sb.setSingleStep(1)

        
        form.addRow("Check Collisions:",             self.check_collisions_cb)
        form.addRow("Collision Check Rate [Hz]:",    self.collision_rate_sb)
        form.addRow("Self collision slowdown [m]:",  self.self_collision_thr_sb)
        form.addRow("Scene collision slowdown [m]:", self.scene_collision_thr_sb)
        form.addRow("Scene collision stop [m]:",     self.scene_collision_stop_sb)
        form.addRow("Prediction Steps:",             self.prediction_steps_sb)
        self.check_collisions_cb.setChecked(True)

        # Read/Write buttons for Servo
        btn_row = QWidget()
        btn_layout = QHBoxLayout(btn_row)
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self._refresh_servo_nodes)
        self.read_servo_btn = QPushButton("Fetch")
        self.read_servo_btn.clicked.connect(self._fetch_servo_params)
        self.write_servo_btn = QPushButton("Apply")
        self.write_servo_btn.clicked.connect(self._apply_servo_params)
        self.pause_servo_btn = QPushButton("Pause")
        self.pause_servo_btn.clicked.connect(self._on_pause_clicked)
        self.pause_servo_btn.setEnabled(False)
        btn_layout.addWidget(self.refresh_btn)
        btn_layout.addWidget(self.read_servo_btn)
        btn_layout.addWidget(self.write_servo_btn)
        btn_layout.addWidget(self.pause_servo_btn)
        layout.addWidget(btn_row)


        blank = QSpacerItem(0, 16, QSizePolicy.Minimum, QSizePolicy.Fixed)
        form.addItem(blank)

        start_servo_grp = QGroupBox("Cartesian Input Topics")
        start_servo_grp.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        layout.addWidget(start_servo_grp)
        form = QFormLayout(start_servo_grp)
        form.setContentsMargins(8, 8, 8, 8)
        form.setSpacing(6)

        robots = self.parent.get_robot_list()

        self.servo_cmd_topic_le = {}
        self.servo_cmd_enabled_cb = {}

        for robot_name in robots:
            line_edit = QLineEdit()
            line_edit.setText(f"/{robot_name}/cmd_vel")
            self.servo_cmd_topic_le[robot_name] = line_edit

            checkbox = QCheckBox("Enabled")
            checkbox.setChecked(True)  # default to enabled
            self.servo_cmd_enabled_cb[robot_name] = checkbox

            # connect toggle to update editability & style
            checkbox.toggled.connect(lambda checked, le=line_edit: self._on_robot_enable_toggled(checked, le))
            # initialize appearance based on default state
            self._on_robot_enable_toggled(checkbox.isChecked(), line_edit)

            row_widget = QWidget()
            row_layout = QHBoxLayout(row_widget)
            row_layout.addWidget(line_edit)
            row_layout.addWidget(checkbox)
            row_layout.setContentsMargins(0, 0, 0, 0)

            form.addRow(f"{robot_name}:", row_widget)

        # Servo Control Buttons (centered)
        btn_row = QWidget()
        btn_layout = QHBoxLayout(btn_row)
        self.start_btn = QPushButton("Start")
        self.start_btn.clicked.connect(self.start_servoing)
        btn_layout.addWidget(self.start_btn)
        self.stop_btn = QPushButton("Stop")
        self.stop_btn.clicked.connect(self.stop_servoing)
        self.stop_btn.setEnabled(False)  # only enabled when servoing is active
        btn_layout.addWidget(self.stop_btn)
        layout.addWidget(btn_row)
        layout.addStretch()

        # Add Geo Constraints Controller button
        self.geo_constraints_btn = QPushButton("Velocity Remapper")
        self.geo_constraints_btn.clicked.connect(self.open_geo_constraints_controller)
        layout.addWidget(self.geo_constraints_btn)
        
    def open_geo_constraints_controller(self):
        if not hasattr(self, '_geo_constraints_window') or self._geo_constraints_window is None:
            self._geo_constraints_window = ControllerGUI(self.node)
        # provide back-reference so geo constraints tab can control servoing
        try:
            self._geo_constraints_window._servo_tab = self
        except Exception:
            pass
        self._geo_constraints_window.show()
        self._geo_constraints_window.raise_()
        self._geo_constraints_window.activateWindow()

    
    def _on_robot_enable_toggled(self, enabled: bool, line_edit: QLineEdit):
        # make non-enabled robot's text field non-editable and gray
        line_edit.setReadOnly(not enabled)
        # keep the widget enabled so the background styling shows clearly
        if not enabled:
            line_edit.setStyleSheet("background-color: #f5f5f5; color: #555555;")
        else:
            line_edit.setStyleSheet("")
    

    def _kill_controller_proc(self):
        if not self._controller_proc:
            return
        try:
            if self._controller_proc.poll() is None:
                try:
                    # kill the entire process group started with start_new_session
                    os.killpg(self._controller_proc.pid, signal.SIGTERM)
                except Exception:
                    try:
                        os.killpg(self._controller_proc.pid, signal.SIGKILL)
                    except Exception as e:
                        logger.warn(f"Failed to kill process group: {e}")
                try:
                    self._controller_proc.wait(timeout=5)
                except Exception:
                    pass
            logger.info("Controller node terminated")
        except Exception as e:
            logger.warn(f"Failed killing controller proc: {e}")
        finally:
            self._controller_proc = None
            self.controller_btn.setText("Start Controller Node")
            self._controller_poll_timer.stop()

    def _poll_controller_proc(self):
        # if process ended externally, update button state
        if self._controller_proc and self._controller_proc.poll() is not None:
            logger.info("Controller node exited")
            self._controller_proc = None
            self.controller_btn.setText("Start Controller Node")
            self._controller_poll_timer.stop()

    def _on_servo_status(self, msg):
        code = msg.code
        text = msg.message or str(code)
        # pick a color
        if code == 0:             # NO_WARNING
            col = "green"
        elif code in (1, 3, 4):   # decelerate warnings
            col = "orange"
        elif code in (2, 5, 6):   # halting conditions
            col = "red"
        else:                     # invalid or unknown
            col = "gray"
        self.status_label.setText(f"{text} ({code})")
        self.status_label.setStyleSheet(f"color: {col};")

    
    def _on_pause_clicked(self):
        node = self.servo_node_cb.currentText().lstrip()
        if not node:
            return
        # toggle this node's paused state
        currently = self._paused_states.get(node, False)
        want = not currently
        # decide data based on current paused state
        #data_str = "{data: %s}" % ("False" if self._paused else "True")
        data_str = "{data: %s}" % ("True" if want else "False")
        svc = f"{node}/pause_servo"
        # fire off ros2 service call
        proc = QProcess(self)
        proc.start("ros2", ["service", "call", svc, "std_srvs/srv/SetBool", data_str])
        # flip our state & update button text
        self._paused_states[node] = want
        self.pause_servo_btn.setText("Unpause" if want else "Pause")
    

    def _update_pause_button_state(self, text: str):
        """Called when servo_node_cb changes; shows Pause or Unpause per‐node."""
        node = text.lstrip('/')
        if not node or text == self._all_nodes_text:
            self.pause_servo_btn.setEnabled(False)
            return
        # only enable if we know about this node
        self.pause_servo_btn.setEnabled(True)
        paused = self._paused_states.get(node, False)
        self.pause_servo_btn.setText("Unpause" if paused else "Pause")
    
    
    def _refresh_servo_nodes(self):
        self._populate_servo_nodes()
    

    def _on_check_collisions_toggled(self):
        try:
            enabled = self.check_collisions_cb.isChecked()
            self.collision_rate_sb.setEnabled(enabled)
            self.self_collision_thr_sb.setEnabled(enabled)
            self.scene_collision_thr_sb.setEnabled(enabled)
            self.scene_collision_stop_sb.setEnabled(enabled)
            self.prediction_steps_sb.setEnabled(enabled)

            if not enabled:
                resp = QMessageBox.warning(
                    self,
                    "Disable Collision Checking",
                    "<html>"
                    "You are about to disable collision checking.<br><br>"
                    "This can be dangerous because the robot will "
                    "not stop before a collision is detected.<br><br>"
                    "Velocity scales will be reduced by a factor of 5 for safety reasons.<br><br>"
                    "Do you really want to do that? <b>Please operate with care!</b>"
                    "</html>",
                    QMessageBox.Yes | QMessageBox.No,
                    QMessageBox.No
                )
                if resp != QMessageBox.Yes:
                    # Revert checkbox back to checked
                    self.check_collisions_cb.setChecked(True)
                    return

                # Save the original values if not already saved
                key = self.servo_node_cb.currentText()
                if key:
                    self._original_scales_by_node[key] = {
                        "linear": self.scale_linear_sb.value(),
                        "rotational": self.scale_rotational_sb.value(),
                    }


                self.scale_linear_sb.setValue(self.scale_linear_sb.value() / 5.0)
                self.scale_rotational_sb.setValue(self.scale_rotational_sb.value() / 5.0)
            else:
                key = self.servo_node_cb.currentText()
                if key in self._original_scales_by_node:
                    self.scale_linear_sb.setValue(self._original_scales_by_node[key]["linear"])
                    self.scale_rotational_sb.setValue(self._original_scales_by_node[key]["rotational"])
                    del self._original_scales_by_node[key]

        except Exception as e:
            logger.error(f"Failed to toggle collision checking: {e}")

    

    def _populate_servo_nodes(self):
        combo = self.servo_node_cb
        prev_select = combo.currentText()
        combo.clear()
        try:
            entries = get_node_names(
                self.node,
                'servo_node',
                exclude_keywords=['_private_'],
                ignore_case=False,
                filter_duplicates=True
            )
            entries.sort()
            self._servo_nodes = entries[:]  # keep list for self._all_nodes_text behavior

            combo.addItem(self._all_nodes_text)      # <-- first option
            combo.addItems(entries)

            if prev_select:
                idx = combo.findText(prev_select)
                if idx >= 0:
                    combo.setCurrentIndex(idx)
                else:
                    combo.setCurrentIndex(0)  # default to self._all_nodes_text
            else:
                combo.setCurrentIndex(0)
        except Exception as e:
            logger.error(f"Failed to populate servo nodes: {e}")

    
    def _clear_servo_nodes(self):
        self._servo_nodes = []
        self.servo_node_cb.clear()
    

    def _stop_current_servo_node(self):
        node_name = self.servo_node_cb.currentText().lstrip('/')
        logger.info(f"Trying to kill {node_name} ... ")
        self.kill_servo_node(node_name)
    
    
    def _fetch_servo_params(self):
        logger.info(f"Fetching params")

        selection = self.servo_node_cb.currentText()
        if selection == self._all_nodes_text:
            if not self._servo_nodes:
                return
            node = self._servo_nodes[0]
        else:
            node = selection.lstrip('/')
            if not node:
                return
        
        params = [
            "moveit_servo.move_group_name",
            "moveit_servo.cartesian_command_in_topic",
            "moveit_servo.apply_twist_commands_about_ee_frame",
            "moveit_servo.scale.linear",
            "moveit_servo.scale.rotational",
            "moveit_servo.publish_period",
            "moveit_servo.max_expected_latency",
            "moveit_servo.override_velocity_scaling_factor",
            "moveit_servo.command_in_type",
            "moveit_servo.joint_topic",
            "moveit_servo.status_topic",
            "moveit_servo.command_out_topic",
            "moveit_servo.check_collisions",
            "moveit_servo.collision_check_rate",
            "moveit_servo.self_collision_proximity_threshold",
            "moveit_servo.scene_collision_proximity_threshold",
            "moveit_servo.scene_collision_proximity_stop_threshold",
            "moveit_servo.prediction_steps"
        ]  
        
        result = get_node_parameters(node, params)

        if result:

            # Update editor fields based on retreived parameters
            self.apply_about_ee_cb.setChecked(
                bool(result["moveit_servo.apply_twist_commands_about_ee_frame"])
            )
            self.scale_linear_sb.setValue(
                float(result["moveit_servo.scale.linear"])
            )
            self.scale_rotational_sb.setValue(
                float(result["moveit_servo.scale.rotational"])
            )
            self.check_collisions_cb.setChecked(
                bool(result["moveit_servo.check_collisions"])
            )
            self.collision_rate_sb.setValue(
                float(result["moveit_servo.collision_check_rate"])
            )
            self.self_collision_thr_sb.setValue(
                float(result["moveit_servo.self_collision_proximity_threshold"])
            )
            self.scene_collision_thr_sb.setValue(
                float(result["moveit_servo.scene_collision_proximity_threshold"])
            )
            self.scene_collision_stop_sb.setValue(
                float(result["moveit_servo.scene_collision_proximity_stop_threshold"])
            )
            self.prediction_steps_sb.setValue(
                float(result["moveit_servo.prediction_steps"])
            )

        else:
            logger.info(f"No parameters retreived – could not be updated!")
    
    
    def _apply_servo_params(self):

        selection = self.servo_node_cb.currentText()
        if selection == self._all_nodes_text:
            target_nodes = [n.lstrip('/') for n in self._servo_nodes]  # strip slashes
            if not target_nodes:
                return
        else:
            node = selection.lstrip('/')
            if not node:
                return
            target_nodes = [node]

        params: dict[str, Any] = {
            "moveit_servo.apply_twist_commands_about_ee_frame":         self.apply_about_ee_cb.isChecked(),
            "moveit_servo.check_collisions":                            self.check_collisions_cb.isChecked(),
            "moveit_servo.scale.linear":                                self.scale_linear_sb.value(),
            "moveit_servo.scale.rotational":                            self.scale_rotational_sb.value(),
            "moveit_servo.collision_check_rate":                        self.collision_rate_sb.value(),
            "moveit_servo.self_collision_proximity_threshold":          self.self_collision_thr_sb.value(),
            "moveit_servo.scene_collision_proximity_threshold":         self.scene_collision_thr_sb.value(),
            "moveit_servo.scene_collision_proximity_stop_threshold":    self.scene_collision_stop_sb.value(),
            "moveit_servo.prediction_steps":                            int(self.prediction_steps_sb.value())
        }

        for node in target_nodes:
            set_node_parameters(node, params, self.node)
            logger.info(f"Applied {len(params)} servo parameters to /{node}")
    
    
    
    def get_servo_active(self):
        return self.servoing_active
    

    def kill_all_servo_nodes(self):
        # best-effort kill of any running servo_node processes
        kill_nodes_by_keyword("servo_node")
    
    
    def start_servoing(self):

         # guard the GUI state
        self._servoing_active = True
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)


        for p in self._servo_launch_procs:
            if p.poll() is None:
                # leave running; could warn or handle differently
                pass
        self._servo_launch_procs.clear()

        robots = self.parent.get_robot_list()
        arm_ids = [] 
        for robot_name in robots: 
            if self.servo_cmd_enabled_cb[robot_name].isChecked():
                arm_ids.append(robot_name)
        
        if not arm_ids:
            QMessageBox.warning(self, "Start Servo", "No enabled robots to start servo for.")
            # revert GUI state since nothing started
            self._servoing_active = False
            self.start_btn.setEnabled(True)
            self.stop_btn.setEnabled(False)
            return
        
        # build launch arguments
        enabled_arms_arg = ",".join(arm_ids)
        cartesian_pairs = []
        for arm_id in arm_ids:
            raw_topic = self.servo_cmd_topic_le[arm_id].text()
            if not raw_topic.startswith("/"):
                raw_topic = "/" + raw_topic
            cartesian_pairs.append(f"{arm_id}:{raw_topic}")
        cartesian_topics_arg = ",".join(cartesian_pairs)

        for arm_id in arm_ids:

            cmd = [
                "ros2",
                "launch",
                "neura_robot_config",
                "servo.launch.py",
                f"enabled_arms:={enabled_arms_arg}",
                f"cartesian_topics:={cartesian_topics_arg}"
            ]
            try:
                proc = subprocess.Popen(cmd)
                self._servo_launch_procs.append(proc)
                #logger.info(f"Started servo launch for {arm_id} with cmd: {' '.join(cmd)}")
                logger.info(f"Started multi-servo launch for arms {enabled_arms_arg} with cmd: {' '.join(cmd)}")
            except Exception as e:
                #QMessageBox.critical(self, "Start Servo", f"Failed to launch servo for {arm_id}: {e}")
                QMessageBox.critical(self, "Start Servo", f"Failed to launch servo for {enabled_arms_arg}: {e}")
                self._servoing_active = False
                self.start_btn.setEnabled(True)
                self.stop_btn.setEnabled(False)
        
        # disable editing while servoing is running
        for line_edit in self.servo_cmd_topic_le.values():
            line_edit.setReadOnly(True)
            line_edit.setStyleSheet("background-color: #f5f5f5; color: #555555;")
        for cb in self.servo_cmd_enabled_cb.values():
            cb.setEnabled(False)
            
        # ensure we have a paused flag for this node
        node = self.servo_node_cb.currentText().lstrip('/')
        self._paused_states.setdefault(node, False)
        self.pause_servo_btn.setEnabled(True)
        self._update_pause_button_state(self.servo_node_cb.currentText())

        self._state_ready = False
        for robot_name in robots:
            topic = f"/{robot_name}/joint_states"
            if self._state_sub:
                self.node.destroy_subscription(self._state_sub)
            self._state_sub = self.node.create_subscription(
                JointState, topic, self._on_first_joint_state_received, 10
            )
        # kickoff our timer loop; it will fire _attempt_switch_when_ready()
        self._wait_timer.start()
    

    def stop_servoing(self):
        # Optionally shut down servo node
        if self.servo_proc and self.servo_proc.poll() is None:
            self.servo_proc.terminate()
            self.servo_proc.wait(5)
            logger.info("Servoing terminated")
        # switch back to standard controller
        QTimer.singleShot(0, self._switch_to_default_controllers)
        self.kill_all_servo_nodes()

        # disable Pause/Unpause whenever servo is stopped
        self.pause_servo_btn.setEnabled(False)
        # and reset its stored state
        node = self.servo_node_cb.currentText().lstrip('/')
        self._paused_states[node] = False
        self.pause_servo_btn.setText("Pause")

        # re-enable cartesian input and enable checkboxes now that servoing stopped
        for line_edit in self.servo_cmd_topic_le.values():
            line_edit.setReadOnly(False)
            line_edit.setStyleSheet("")
        for cb in self.servo_cmd_enabled_cb.values():
            cb.setEnabled(True)

        # new GUI state handling
        self._servoing_active = False
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)

    def _on_first_joint_state_received(self, msg: JointState):
        # just the arrival of the first RobotState is enough
        # once we see one complete RobotState we know planning_scene monitor
        # now has the full joint state
        self._state_ready = True
    
    
    def _switch_command_type_and_controllers(self):
        # call switch_command_type on each namespace
        robots = self.parent.get_robot_list()

        for robot_name in robots:
            self._switch_command_type(robot_name, command_type=1)
        # then swap controllers
        self._switch_to_servo_controllers()
    

    def _switch_command_type(self, ns: str, command_type: int = 1, attempt: int = 1):
        client = self._cmd_type_clients[ns]
        # try up to 3 times
        if not client.wait_for_service(timeout_sec=5.0):
            if attempt < 3:
                logger.warning(f"{ns}: switch_command_type service unavailable (attempt {attempt}), retrying in 1s")
                QTimer.singleShot(1000,
                    lambda: self._switch_command_type(ns, command_type, attempt + 1)
                )
            else:
                logger.error(f"{ns}: switch_command_type service unavailable after 3 attempts")
            return

        req = ServoCommandType.Request()
        req.command_type = command_type
        future = client.call_async(req)
        future.add_done_callback(lambda fut, ns=ns, ct=command_type: 
                                    self._on_switch_command_type_done(fut, ns, ct))

    

    def _on_switch_command_type_done(self, fut, ns, command_type):
        try:
            result = fut.result()
            if result.success:
                logger.info(f"{ns}: command_type set to {command_type}")
                # Now it's safe to start publishing input
                self.servo_start.emit(ns)  # or enable input stream
            else:
                logger.warning(f"{ns}: Failed to set command_type")
        except Exception as e:
            logger.error(f"{ns}: Service call failed: {e}")



    def _switch_to_servo_controllers(self):
        for ns, client in self._svc_clients.items():
            if not client.wait_for_service(timeout_sec=2.0):
                logger.error(f"{ns}: controller switch service unavailable")
                continue

            req = SwitchController.Request()
            req.start_controllers = ["joint_pos_forward_command"]
            req.stop_controllers  = ["joint_pos_scaled_trajectory"]
            # if you want best-effort switching
            req.strictness = SwitchController.Request.BEST_EFFORT
            ctrl = req.start_controllers[0]

            self.servoing_active = True
            self._populate_servo_nodes()

            future = client.call_async(req)
            future.add_done_callback(lambda fut, ns=ns: self._on_controller_switch_done(fut, ns, ctrl))


    def _on_controller_switch_done(self, future, ns, ctrl):
        try:
            resp = future.result()
            logger.info(f"{ns}: Switched to Controller: {ctrl}")
        
        except Exception as e:
            logger.error(f"{ns}: controller switch failed: {e}")



    def _switch_to_default_controllers(self):
        for ns, client in self._svc_clients.items():
            if not client.wait_for_service(timeout_sec=2.0):
                continue
            req = SwitchController.Request()
            req.start_controllers = ["joint_pos_scaled_trajectory"]
            req.stop_controllers  = ["joint_pos_forward_command"]
            req.strictness = SwitchController.Request.BEST_EFFORT

            self.servoing_active = False
            self._clear_servo_nodes()
            
            ctrl = req.start_controllers[0]
            future = client.call_async(req)
            future.add_done_callback(lambda fut, ns=ns: self._on_controller_switch_done(fut, ns, ctrl))
    
    def _attempt_switch_when_ready(self):
        # wait until RobotState arrives, then switch
        if not self._state_ready:
            return
        self._wait_timer.stop()
        # tear down the subscription so we don't keep getting messages
        if self._state_sub:
            self.node.destroy_subscription(self._state_sub)
            self._state_sub = None
        # now safe to flip MoveIt Servo command type & controllers
        logger.warn(f"Switch command type and controllers")
        self._switch_command_type_and_controllers()