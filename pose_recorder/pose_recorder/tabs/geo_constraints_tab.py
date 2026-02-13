from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QLabel,
    QPushButton, QGroupBox, QFormLayout, QCheckBox, QLineEdit, QHBoxLayout, QMessageBox, QComboBox
)
from PySide6.QtCore import QTimer, Qt
from std_msgs.msg import String, Bool, ColorRGBA
from pose_recorder.trajectory_commander import TrajectoryCommander
from pose_recorder.utils import deg_to_rad
import subprocess
import os
import signal
import tf2_ros
import tf2_geometry_msgs 
import random
from geometry_msgs.msg import Vector3Stamped, Pose
from tf2_ros import TransformException
import rclpy
from rclpy.time import Time
from rclpy.duration import Duration
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, ObjectColor, PlanningScene
from shape_msgs.msg import SolidPrimitive
from tf_transformations import quaternion_from_euler 

class ControllerGUI(QWidget):
    def __init__(self, ros_node):
        """
        :param ros_node: rclpy Node shared from pose_recorder_gui
        """
        super().__init__()
        self.node = ros_node
        self.setWindowTitle("Geometric Constraints Controller")
        

        self.spawn_pose = Pose()
        self.spawn_dims = None
        self.aimed_pose = Pose()
        self.obj_pose_wrt_ee1 = Pose()
        #Subprocess management
        self._controller_proc: subprocess.Popen | None = None
        self._controller_poll_timer = QTimer(self)
        self._controller_poll_timer.setInterval(500)
        self._controller_poll_timer.timeout.connect(self._poll_controller_proc)

        #Launch parameters
        self.base_state = None  # "fixed_rt", "mirroring", "none", optionally with "_with_line" suffix
        # Create ROS publishers if they don’t exist already
        self.robot_pub = self.node.create_publisher(String, '/selected_robot', 10)
        self.surface_normal_pub = self.node.create_publisher(String, '/surface_normal', 10)
        self.state_pub = self.node.create_publisher(String, '/motion_type', 10)  # New publisher for state
        # Control Type publisher
        self.control_type_pub = self.node.create_publisher(Bool, '/control_type', 10)
        # Publisher to request saving/unfreezing robots
        self.rescue_pub = self.node.create_publisher(Bool, '/rescue_robots', 10)
        # Publisher for attaching existing collision objects
        self.attach_box_pub = self.node.create_publisher(AttachedCollisionObject, "/attached_collision_object", 10)
        self.spawn_obj_pub = self.node.create_publisher(CollisionObject, "/collision_object", 10)
        self.scene_pub = self.node.create_publisher(PlanningScene, "/planning_scene", 10)
        # Create widgets
        self.publish_button = QPushButton("Publish Selection")
        self.publish_button.clicked.connect(self.publish_selection)

        # Checkboxes for robots
        self.neura_1_checkbox = QCheckBox("neura_1")
        self.neura_2_checkbox = QCheckBox("neura_2")
        self.neura_1_checkbox.setChecked(True)
        self.neura_2_checkbox.setChecked(True)
        self.neura_1_checkbox.stateChanged.connect(self.update_button_states)
        self.neura_2_checkbox.stateChanged.connect(self.update_button_states)

        # Input fields for surface normal vector
        self.surface_normal_x = QLineEdit()
        self.surface_normal_x.setPlaceholderText("Enter X component")
        self.surface_normal_x.setText("0.0")
        self.surface_normal_x.textChanged.connect(self._on_normal_vector) 
        self.surface_normal_y = QLineEdit()
        self.surface_normal_y.setPlaceholderText("Enter Y component")
        self.surface_normal_y.setText("0.0")
        self.surface_normal_y.textChanged.connect(self._on_normal_vector)  # Auto-publish on change
        self.surface_normal_z = QLineEdit()
        self.surface_normal_z.setPlaceholderText("Enter Z component")
        self.surface_normal_z.setText("0.0")    
        self.surface_normal_z.textChanged.connect(self._on_normal_vector)  # Auto-publish on change


        # Buttons for additional functionality
        # Use checkboxes for Fixed RT and Mirroring (behave like toggles)
        self.fixed_rt_button = QCheckBox("Fixed Distance")
        self.fixed_rt_button.setEnabled(True)
        self.fixed_rt_button.setChecked(True)
        self.fixed_rt_button.stateChanged.connect(self._on_fixed_rt_toggled)

        self.sphere_button = QCheckBox("Rotate the object")
        self.sphere_button.setEnabled(True)
        self.sphere_button.toggled.connect(self._on_sphere_toggled)

        self.mirroring_button = QCheckBox("Mirroring")
        self.mirroring_button.setEnabled(True)
        self.mirroring_button.stateChanged.connect(self._on_mirroring_toggled)

        # Line motion button
        self.line_button = QPushButton("Enable Line Motion")
        self.line_button.setEnabled(True)
        self.line_button.clicked.connect(self.toggle_line_motion)

        #Feedback Combo Box
        self.control_type = QComboBox()
        self.control_type.addItem("Master-Slave Controll", "master_slave")
        self.control_type.addItem("Sensor Velocity Controll", "sensor_velocity")
        self.control_type.setCurrentIndex(0)  # default to Master-Slave
        self.control_type.currentIndexChanged.connect(self.publish_selection)
        
        self.spawn_object_btn = QPushButton("Spawn Test Object")
        self.spawn_object_btn.setEnabled(True)
        self.spawn_object_btn.setCheckable(True)
        self.spawn_object_btn.setStyleSheet("""
            QPushButton {
                background-color: #2d2f31;       /* off */
                color: white;
                border: 1px solid #3d4043;
                border-radius: 8px;
                padding: 6px 12px;
            }
            QPushButton:hover {
                background-color: #3a3d40;
            }
            QPushButton:checked {
                background-color: #0e7a0d;       /* ON color */
                border-color: #0d6c0c;
            }
            QPushButton:checked:hover {
                background-color: #0c6c0b;
            }
            QPushButton:disabled {
                background-color: #55595c;
                color: #bfc5c9;
                border-color: #65686b;
            }
            """)
        self.spawn_object_btn.clicked.connect(self._on_spawn_object_clicked)

        self.obj_fetched_btn = QPushButton("Object is fetched")
        self.obj_fetched_btn.setEnabled(True)
        self.obj_fetched_btn.setCheckable(True)
        self.obj_fetched_btn.setStyleSheet("""
            QPushButton {
                background-color: #2d2f31;       /* off */
                color: white;
                border: 1px solid #3d4043;
                border-radius: 8px;
                padding: 6px 12px;
            }
            QPushButton:hover {
                background-color: #3a3d40;
            }
            QPushButton:checked {
                background-color: #0e7a0d;       /* ON color */
                border-color: #0d6c0c;
            }
            QPushButton:checked:hover {
                background-color: #0c6c0b;
            }
            QPushButton:disabled {
                background-color: #55595c;
                color: #bfc5c9;
                border-color: #65686b;
            }
            """)
        self.obj_fetched_btn.toggled.connect(self._on_object_fetched_toggled)

        # Buttons to move robots to each other's pose
        self.move_to_robot1_btn = QPushButton("Move to Robot 1 Pose")
        self.move_to_robot1_btn.clicked.connect(self.move_to_robot1_pose)

        self.move_to_robot2_btn = QPushButton("Move to Robot 2 Pose")
        self.move_to_robot2_btn.clicked.connect(self.move_to_robot2_pose)

        # Trajectory commander for PTP moves
        try:
            self.traj_commander = TrajectoryCommander(self.node)
        except Exception:
            self.traj_commander = None

        # TF2: buffer + listener (spin thread so transforms arrive)
        try:
            self.tf_buffer = tf2_ros.Buffer()
            # TransformListener attaches to node and spins internally when spin_thread=True
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node, spin_thread=True)
        except Exception as e:
            self.tf_buffer = None
            self.node.get_logger().warn(f"Failed to create TF buffer/listener: {e}")

        # Layout
        layout = QVBoxLayout()

        # Top Geo Constraint group (Surface definition + Line Motion button)
        geo_group = QGroupBox("Geo Constraint")
        geo_layout = QVBoxLayout()

        # single row: label + X Y Z side-by-side
        surface_row = QWidget()
        surface_row_layout = QHBoxLayout(surface_row)
        surface_row_layout.setContentsMargins(0, 0, 0, 0)
        surface_row_layout.addWidget(QLabel("Surface Normal vector:"))
        surface_row_layout.addWidget(QLabel('X:')); surface_row_layout.addWidget(self.surface_normal_x)
        surface_row_layout.addWidget(QLabel('Y:')); surface_row_layout.addWidget(self.surface_normal_y)
        surface_row_layout.addWidget(QLabel('Z:')); surface_row_layout.addWidget(self.surface_normal_z)

        # New UI: robot/plane comboboxes + apply button (same line)
        self.surface_robot_combo = QComboBox()
        self.surface_robot_combo.addItem("robot 1", "neura_1")
        self.surface_robot_combo.addItem("robot 2", "neura_2")
        self.surface_plane_combo = QComboBox()
        self.surface_plane_combo.addItem("XY plane", "xy")
        self.surface_plane_combo.addItem("XZ plane", "xz")
        self.surface_plane_combo.addItem("YZ plane", "yz")
        self.apply_surface_btn = QPushButton("Apply Surface")
        self.apply_surface_btn.clicked.connect(self.apply_surface)

        # Compact controls row: use two small QFormLayouts side-by-side so labels sit close to combos
        surface_controls = QWidget()
        surface_controls_layout = QHBoxLayout(surface_controls)
        surface_controls_layout.setContentsMargins(0, 0, 0, 0)
        surface_controls_layout.setSpacing(8)

        # Robot form (compact)
        robot_form_widget = QWidget()
        robot_form_layout = QFormLayout(robot_form_widget)
        robot_form_layout.setContentsMargins(0, 0, 0, 0)
        robot_form_layout.setSpacing(4)
        # center label text (makes label sit close and centered relative to the field)
        robot_form_layout.setLabelAlignment(Qt.AlignCenter)
        robot_form_layout.addRow("Robot:", self.surface_robot_combo)

        # Plane form (compact)
        plane_form_widget = QWidget()
        plane_form_layout = QFormLayout(plane_form_widget)
        plane_form_layout.setContentsMargins(0, 0, 0, 0)
        plane_form_layout.setSpacing(4)
        # center label text
        plane_form_layout.setLabelAlignment(Qt.AlignCenter)
        plane_form_layout.addRow("Plane:", self.surface_plane_combo)

        surface_controls_layout.addWidget(robot_form_widget)
        surface_controls_layout.addWidget(plane_form_widget)
        surface_controls_layout.addWidget(self.apply_surface_btn)
        surface_controls_layout.addStretch(1)

        # center the small form widgets vertically (and horizontally within their area)
        surface_controls_layout.setAlignment(robot_form_widget, Qt.AlignHCenter | Qt.AlignVCenter)
        surface_controls_layout.setAlignment(plane_form_widget, Qt.AlignHCenter | Qt.AlignVCenter)

        # New: Surface Definition group (contains surface row + the new controls row)
        surface_def_group = QGroupBox("Planar Motion")
        surface_def_layout = QVBoxLayout()
        surface_def_layout.setContentsMargins(6, 6, 6, 6)
        surface_def_layout.addWidget(surface_row)
        surface_def_layout.addWidget(surface_controls)   # <-- added row with comboboxes + button
        surface_def_group.setLayout(surface_def_layout)

        # New: Constraints group (contains mode toggles and line motion button)
        constraints_group = QGroupBox("Constraints")
        constraints_layout = QVBoxLayout()
        # modes row (Fixed RT, Mirroring, Sphere)
        modes = QWidget()
        modes_layout = QHBoxLayout(modes)
        modes_layout.setContentsMargins(0, 0, 0, 0)
        modes_layout.addWidget(self.fixed_rt_button)
        modes_layout.addWidget(self.mirroring_button)
        modes_layout.addWidget(self.sphere_button)
        constraints_layout.addWidget(modes)
        # put the existing Line Motion button directly under the modes row
        constraints_layout.addWidget(self.line_button)
        constraints_group.setLayout(constraints_layout)

        # Add the two inner groups to the geo_group
        geo_layout.addWidget(surface_def_group)
        geo_layout.addWidget(constraints_group)
        geo_group.setLayout(geo_layout)

        # Group for robot checkboxes
        robot_group = QGroupBox("Robot Selection")
        robot_layout = QHBoxLayout()
        robot_layout.addWidget(self.neura_1_checkbox)
        robot_layout.addWidget(self.neura_2_checkbox)
        robot_group.setLayout(robot_layout)

        feedback_group = QGroupBox("Feedback Type")
        feedback_layout = QFormLayout(feedback_group)
        feedback_layout.addRow('Feedback Type:', self.control_type)

        layout.addWidget(geo_group)
        layout.addWidget(robot_group)
        layout.addWidget(feedback_group)
        # line_button is now inside geo_group above

        # Place the move buttons side-by-side in a horizontal row
        obj_row = QWidget()
        obj_row_layout = QHBoxLayout(obj_row)
        obj_row_layout.setContentsMargins(0, 0, 0, 0)
        obj_row_layout.addWidget(self.spawn_object_btn)
        obj_row_layout.addWidget(self.obj_fetched_btn)

        # Group Object Spawn + Obj is Fetched and Evaluate row under "User Study"
        user_study_group = QGroupBox("User Study")
        user_study_layout = QVBoxLayout()
        user_study_layout.setContentsMargins(6, 6, 6, 6)
        user_study_layout.addWidget(obj_row)

        # Evaluate row: button + two read-only fields for errors
        self.evaluate_btn = QPushButton("Evaluate")
        self.evaluate_btn.setEnabled(True)
        self.evaluate_btn.clicked.connect(self.evaluate_error)

        self.error_distance_x = QLineEdit()
        self.error_distance_x.setPlaceholderText("Error in x")
        self.error_distance_x.setReadOnly(True)
        self.error_distance_y = QLineEdit()
        self.error_distance_y.setPlaceholderText("Error in y")
        self.error_distance_y.setReadOnly(True)

        evaluate_row = QWidget()
        evaluate_row_layout = QHBoxLayout(evaluate_row)
        evaluate_row_layout.setContentsMargins(0, 0, 0, 0)
        evaluate_row_layout.addWidget(self.evaluate_btn)
        evaluate_row_layout.addWidget(self.error_distance_x)
        evaluate_row_layout.addWidget(self.error_distance_y)
        evaluate_row_layout.addStretch(1)

        user_study_layout.addWidget(evaluate_row)
        user_study_group.setLayout(user_study_layout)
        layout.addWidget(user_study_group)

        # Save Robots button
        self.rescue_robots_btn = QPushButton("Rescue Robots")
        self.rescue_robots_btn.clicked.connect(self.publish_rescue_robots)
        layout.addWidget(self.rescue_robots_btn)

        self.setLayout(layout)

        self.controller_btn = QPushButton("Start Controller Node")
        self.controller_btn.clicked.connect(self._on_controller_btn_clicked)
        layout.addWidget(self.controller_btn)

    def publish_selection(self):
        # Determine selected robots based on checkboxes
        selected_robots = []
        if self.neura_1_checkbox.isChecked():
            selected_robots.append("neura_1")
        if self.neura_2_checkbox.isChecked():
            selected_robots.append("neura_2")

        robot = "-".join(selected_robots) if selected_robots else "none"


        robot_msg = String()
        robot_msg.data = robot
        self.robot_pub.publish(robot_msg)
        self.node.get_logger().info(f"[GUI] Published robot: {robot}")

        # Get surface normal vector components
        x = self.surface_normal_x.text()
        y = self.surface_normal_y.text()
        z = self.surface_normal_z.text()
        surface_normal = f"{x},{y},{z}"

        surface_normal_msg = String()
        surface_normal_msg.data = surface_normal
        self.surface_normal_pub.publish(surface_normal_msg)
        self.node.get_logger().info(f"[GUI] Published surface normal: {surface_normal}")

        # Publish control type
        control_type_value = self.control_type.currentData() == "master_slave"   
        control_type_msg = Bool()
        control_type_msg.data = control_type_value
        self.control_type_pub.publish(control_type_msg)
        self.node.get_logger().info(f"[GUI] Published control type: {'master_slave' if control_type_value else 'sensor_velocity'}")

        # Publish the current state (base mode + optional "_with_line" suffix)
        if self.fixed_rt_button.isChecked():
            self.base_state = "fixed_rt"
        elif self.mirroring_button.isChecked():
            self.base_state = "mirroring"
        elif self.sphere_button.isChecked():
            self.base_state = "sphere"
        else:
            self.base_state = "none"

        eps = 1e-6  # pick your sensitivity

        vx = self.to_float(x)
        vy = self.to_float(y)
        vz = self.to_float(z)
        # If surface normal provided, force fixed_rt and ensure checkbox state
        if (abs(vx) > eps) or (abs(vy) > eps) or (abs(vz) > eps):
            self.base_state = "fixed_rt"
            # programmatically set checkboxes to reflect this
            self.fixed_rt_button.setChecked(True)
            self.mirroring_button.setChecked(False)
            self.sphere_button.setChecked(False)
            # reset line button visual to default (it remains a QPushButton)
            self.line_button.setText("Enable Line Motion")

        # append line modifier if enabled
        if self.line_button.text() == "Disable Line Motion" and self.base_state != "none":
            state_payload = f"{self.base_state}_with_line"
        else:
            state_payload = self.base_state

        state_msg = String()
        state_msg.data = state_payload
        self.state_pub.publish(state_msg)
        self.node.get_logger().info(f"[GUI] Published state: {state_msg.data}")

    def publish_rescue_robots(self):
        try:
            msg = Bool()
            msg.data = True
            self.rescue_pub.publish(msg)
            self.node.get_logger().info("[GUI] Published rescue_robots: True")
        except Exception as e:
            self.node.get_logger().error(f"publish_rescue_robots failed: {e}")
    

    def _on_controller_btn_clicked(self):
    # toggle starting/killing the velocity_remapper node
        try:
            if self._controller_proc is None or self._controller_proc.poll() is not None:
                # start
                cmd = ["ros2", "launch", "velocity_remapper", "velocity_remapper.launch.py", f"mode:={self.base_state}", 
                       f"neura_1_enabled:={self.neura_1_checkbox.isChecked()}", f"neura_2_enabled:={self.neura_2_checkbox.isChecked()}",
                       f"surface_normal_x:={self.surface_normal_x.text()}", f"surface_normal_y:={self.surface_normal_y.text()}", f"surface_normal_z:={self.surface_normal_z.text()}",
                       f"master_slave_control:={self.control_type.currentData() == 'master_slave'}"] 
                try:
                    # start the controller in a new session so we can kill the whole process group
                    self._controller_proc = subprocess.Popen(cmd, start_new_session=True)
                    self.node.get_logger().info(f"Started controller node with pid {self._controller_proc.pid}")
                    self.controller_btn.setText("Kill Controller Node")
                    self._controller_poll_timer.start()
                except Exception as e:
                    QMessageBox.critical(self, "Start Controller", f"Failed to start controller node: {e}")
                    self._controller_proc = None
            else:
                # kill
                self._kill_controller_proc()
                #self._kill_controller()
        except Exception as e:
            self.node.get_logger().error(f"Controller button handler failed: {e}")

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
                        self.node.get_logger().warn(f"Failed to kill process group: {e}")
                try:
                    self._controller_proc.wait(timeout=5)
                except Exception:
                    pass
            self.node.get_logger().info("Controller node terminated")
        except Exception as e:
            self.node.get_logger().warn(f"Failed killing controller proc: {e}")
        finally:
            self._controller_proc = None
            self.controller_btn.setText("Start Controller Node")
            self._controller_poll_timer.stop()

    def _poll_controller_proc(self):
        # if process ended externally, update button state
        if self._controller_proc and self._controller_proc.poll() is not None:
            self.node.get_logger().info("Controller node exited")
            self._controller_proc = None
            self.controller_btn.setText("Start Controller Node")
            self._controller_poll_timer.stop()

    def update_button_states(self):
        # Enable controls only if both robots are selected
        is_enabled = self.neura_1_checkbox.isChecked() and self.neura_2_checkbox.isChecked()
        self.publish_selection()  # auto-publish on change
        if not self._on_normal_vector():
            self.fixed_rt_button.setEnabled(is_enabled)
            self.mirroring_button.setEnabled(is_enabled)
            self.sphere_button.setEnabled(is_enabled)
            self.line_button.setEnabled(is_enabled)
        # If enabling, keep existing checked states; if disabling, clear toggles
        #if not is_enabled:
        #    self.fixed_rt_button.setChecked(False)
        #    self.mirroring_button.setChecked(False)

    def to_float(self, s: str) -> float:
        s = (s or "").strip()
        # allow comma decimal separators just in case
        s = s.replace(",", ".")
        try:
            v = float(s)
            # treat NaN/Inf as 0 for safety
            if not (abs(v) < float('inf')):
                return 0.0
            return v
        except ValueError:
            return 0.0
        
    def attach_existing_box(self, link="neura_1/flange", obj_id="box_1"):
        aco = AttachedCollisionObject()
        aco.link_name = link
        aco.object.id = obj_id
        aco.object.operation = CollisionObject.ADD      # attach existing object by id
        # optional: allow gripper links to touch
        aco.touch_links = [link, "neura_1/link6", "neura_1/link7", "neura_2/flange"]
        self.attach_box_pub.publish(aco)
        self.node.get_logger().info(f"[GUI] Attached existing '{obj_id}' to {link}")

    def detach_object(self, link="neura_1/flange", obj_id="box_1"):
        aco = AttachedCollisionObject()
        aco.link_name = link
        aco.object.id = obj_id
        aco.object.operation = CollisionObject.REMOVE   # <-- detach
        # touch_links not needed for detach
        self.attach_box_pub.publish(aco)
        self.node.get_logger().info(f"[GUI] Detached '{obj_id}' from {link}")
    
    def spawn_test_object(self, box_id="box_1", x=0.0, y=0.0, z=0.9, dimensions=[0.4, 0.2, 0.1], color=(0.1, 0.7, 0.2, 0.9), rpy=(0.0, 0.0, 0.0)):
        roll, pitch, yaw = rpy
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

        co = CollisionObject()
        co.id = box_id
        co.header.frame_id = "world_link"
        co.operation = CollisionObject.ADD
        # Define a simple box shape
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = dimensions  # [x_size, y_size, z_size]
        box_pose = Pose()
        box_pose.position.x = x
        box_pose.position.y = y
        box_pose.position.z = z
        box_pose.orientation.x = qx
        box_pose.orientation.y = qy
        box_pose.orientation.z = qz
        box_pose.orientation.w = qw

        co.primitives.append(box)
        co.primitive_poses.append(box_pose)

        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects.append(co)

        
        oc = ObjectColor()
        oc.id = box_id
        oc.color = ColorRGBA(r=float(color[0]),
                            g=float(color[1]),
                            b=float(color[2]),
                            a=float(color[3]))  # set a<1.0 for transparency

        ps.object_colors.append(oc)

        self.node.get_logger().info(f"[GUI] Spawning colored object '{box_id}' in world_link")
        self.scene_pub.publish(ps)
        # co.primitives.append(box)
        # co.primitive_poses.append(box_pose)
        # self.node.get_logger().info(f"[GUI] Spawning test object '{box_id}' in world_link frame")
        # self.spawn_obj_pub.publish(co)


    def _on_normal_vector(self):
        for field in [self.surface_normal_x, self.surface_normal_y, self.surface_normal_z]:
            if float(field.text()) != 0:
                # if any component is non-zero, force Fixed RT mode
                self.fixed_rt_button.setChecked(True)
                self.fixed_rt_button.setEnabled(False)
                self.mirroring_button.setChecked(False)
                self.mirroring_button.setEnabled(False)
                self.sphere_button.setChecked(False)
                self.sphere_button.setEnabled(False)
                self.line_button.setEnabled(False)
                self.node.get_logger().info("[GUI] Non-zero surface normal detected, forcing Fixed RT mode")
                self.publish_selection()
                return True   

        self.fixed_rt_button.setEnabled(True)
        self.mirroring_button.setEnabled(True)
        self.sphere_button.setEnabled(True)
        self.line_button.setEnabled(True)    
        self.publish_selection() 
        return False           


    def _on_fixed_rt_toggled(self, state):
        checked = bool(state)
        if checked:
            # when Fixed RT enabled, disable Mirroring and Line Motion
            self.mirroring_button.setChecked(False)
            self.sphere_button.setChecked(False)
            self.node.get_logger().info("[GUI] Fixed RT enabled")
        else:
            # re-enable mirroring and line button based on robot selection
            en = self.neura_1_checkbox.isChecked() and self.neura_2_checkbox.isChecked()
            self.mirroring_button.setEnabled(en)
            self.node.get_logger().info("[GUI] Fixed RT disabled")
        self.publish_selection()

    def _on_mirroring_toggled(self, state):
        checked = bool(state)
        if checked:
            # when Mirroring enabled, disable Fixed RT and Line Motion
            self.fixed_rt_button.setChecked(False)
            self.sphere_button.setChecked(False)
            self.node.get_logger().info("[GUI] Mirroring enabled")
        else:
            # re-enable fixed_rt and line button based on robot selection
            en = self.neura_1_checkbox.isChecked() and self.neura_2_checkbox.isChecked()
            self.fixed_rt_button.setEnabled(en)
            self.node.get_logger().info("[GUI] Mirroring disabled")
        self.publish_selection()
    
    def _on_sphere_toggled(self, checked: bool):
        if checked:
            # when Sphere enabled, disable Fixed RT and Mirroring and Line Motion
            self.fixed_rt_button.setChecked(False)
            self.mirroring_button.setChecked(False)
            self.node.get_logger().info("[GUI] Sphere Constraint enabled")
            #self.control_type.setCurrentIndex(1)  # switch to Sensor Velocity control
            #self.sphere_button.setText("Object is fetched")
        else:
            # re-enable fixed_rt and mirroring and line button based on robot selection
            en = self.neura_1_checkbox.isChecked() and self.neura_2_checkbox.isChecked()
            self.fixed_rt_button.setEnabled(en)
            #self.sphere_button.setText("Object is released")
            self.node.get_logger().info("[GUI] Sphere Constraint disabled")
        self.publish_selection()
        

    def toggle_line_motion(self):
        if self.line_button.text() == "Enable Line Motion":
            self.line_button.setText("Disable Line Motion")
            self.node.get_logger().info("[GUI] Line motion enabled: published 'line'")
        else:
            self.line_button.setText("Enable Line Motion")
            self.node.get_logger().info("[GUI] Line motion disabled: published 'none'")
        self.publish_selection()

    def _on_object_fetched_toggled(self, checked: bool):
        if checked:
            self.attach_existing_box(link="neura_1/flange", obj_id="box_1")
            tf_world_to_ee1 = self.tf_buffer.lookup_transform("neura_1/flange", "world_link", Time())
            self.obj_pose_wrt_ee1 = tf2_geometry_msgs.do_transform_pose(self.spawn_pose, tf_world_to_ee1)
        else:
            self.detach_object(link="neura_1/flange", obj_id="box_1")

    def _on_spawn_object_clicked(self, checked: bool):
        if checked:
            x = random.uniform(-0.2, 0.2)
            y = random.uniform(-0.4, 0.4)
            self.spawn_test_object(box_id="box_1", x=x, y=y, rpy = (0.0, 0.0, 0.0))
            self.spawn_test_object(box_id="aimed_position", x=0.0, y=0.0, z=0.9, dimensions=[0.2, 0.4, 0.1], color=(0.9, 0.0, 0.0, 0.9))
            self.spawn_pose.position.x = x
            self.spawn_pose.position.y = y
            self.spawn_pose.position.z = 0.9
            self.aimed_pose.position.x = 0.0
            self.aimed_pose.position.y = 0.0
            self.aimed_pose.position.z = 0.9
            self.spawn_dims = [0.4, 0.2, 0.1]
        else:
            # Optionally implement object removal here
            pass

    # New: handler for Apply Surface button
    def apply_surface(self):
        # Build a Vector3Stamped in the chosen robot EE frame, transform to world_link and update fields
        robot = self.surface_robot_combo.currentData()
        plane = self.surface_plane_combo.currentData()
        # integer normal in EE frame (user requested integers)
        if plane == "xy":
            nx, ny, nz = 0, 0, 1
        elif plane == "xz":
            nx, ny, nz = 0, 1, 0
        else:  # yz
            nx, ny, nz = 1, 0, 0

        # prepare stamped vector in robot EE frame
        vec = Vector3Stamped()
        # assume an EE frame naming convention — adjust if your system uses different frame ids
        ee_frame = f"{robot}/ee_tcp"
        vec.header.frame_id = ee_frame
        try:
            vec.header.stamp = self.node.get_clock().now().to_msg()
        except Exception:
            # if node clock not available, leave stamp unset
            pass
        vec.vector.x = float(nx)
        vec.vector.y = float(ny)
        vec.vector.z = float(nz)

        # Transform to world_link if tf buffer available
        if self.tf_buffer is None:
            self.node.get_logger().warn("[GUI] tf buffer not available, applying EE-frame normal directly")
            tx, ty, tz = vec.vector.x, vec.vector.y, vec.vector.z
        else:
            try:
                # small timeout for transform lookup
                transformed = self.tf_buffer.transform(vec, 'world_link', timeout=Duration(seconds=1.0))
                #transformed = self.node.tf_buffer.transform(vec, 'world_link', timeout=Duration(seconds=1.0))
                tx, ty, tz = transformed.vector.x, transformed.vector.y, transformed.vector.z
            except TransformException as e:
                self.node.get_logger().error(f"[GUI] transform failed: {e}")
                # fall back to EE-frame values
                tx, ty, tz = vec.vector.x, vec.vector.y, vec.vector.z
            except Exception as e:
                self.node.get_logger().error(f"[GUI] unexpected transform error: {e}")
                tx, ty, tz = vec.vector.x, vec.vector.y, vec.vector.z

        # set the UI text fields (use formatted floats)
        self.surface_normal_x.setText(f"{tx:.3f}")
        self.surface_normal_y.setText(f"{ty:.3f}")
        self.surface_normal_z.setText(f"{tz:.3f}")
        self.node.get_logger().info(f"[GUI] Applied surface: robot={robot}, plane={plane}, world_normal={tx:.6f},{ty:.6f},{tz:.6f}")
        # trigger normal-vector handling (updates UI state)
        self._on_normal_vector()

    def _get_joint_positions_rad(self, joint_pose):
        # joint_pose expected to have 'joint_names' and 'joint_angles_degrees' dict
        names = joint_pose.get('joint_names', [])
        angles_deg = joint_pose['joint_angles_degrees'].values()
        return names,[deg_to_rad(angle) for angle in angles_deg] 

    def move_to_robot1_pose(self):
        # Move robot1 to robot2's current pose
        if not self.traj_commander:
            self.node.get_logger().error("TrajectoryCommander not available")
            return
        try:
            # stop servoing (if available)
            if hasattr(self, '_servo_tab') and self._servo_tab:
                try:
                    self._servo_tab.stop_servoing()
                except Exception as e:
                    self.node.get_logger().warn(f"Failed to stop servoing: {e}")

            src = 'neura_2'
            tgt = 'neura_1'
            all_names = []
            all_positions = []
            joint_pose_tgt = self.node._get_joint_state(tgt)
            joint_pose_src = self.node._get_joint_state(src)
            if not joint_pose_tgt:
                self.node.get_logger().error(f"Failed to get joint state for {tgt}")
                return
            _, positions_tgt = self._get_joint_positions_rad(joint_pose_tgt)
            all_names.extend(joint_pose_tgt['joint_names'])
            all_positions.extend(positions_tgt)
            all_names.extend(joint_pose_src['joint_names'])
            positions_tgt[::2] = [-x for x in positions_tgt[::2]]
            all_positions.extend(positions_tgt)
            self.node.get_logger().info(f"Moving {src} to {tgt} pose")
            self.traj_commander.send_ptp_trajectory(all_names, all_positions)

            # monitor execution and restart servoing when done
            if hasattr(self.traj_commander, 'is_done'):
                timer = QTimer(self)
                timer.setInterval(200)
                def _poll():
                    try:
                        if self.traj_commander.is_done():
                            timer.stop()
                            if hasattr(self, '_servo_tab') and self._servo_tab:
                                try:
                                    self._servo_tab.start_servoing()
                                except Exception as e:
                                    self.node.get_logger().warn(f"Failed to start servoing: {e}")
                    except Exception as e:
                        timer.stop()
                        self.node.get_logger().warn(f"Error while monitoring trajectory: {e}")
                timer.timeout.connect(_poll)
                timer.start()

        except Exception as e:
            self.node.get_logger().error(f"move_to_robot1_pose failed: {e}")

    def move_to_robot2_pose(self):
        # Move robot2 to robot1's current pose
        if not self.traj_commander:
            self.node.get_logger().error("TrajectoryCommander not available")
            return
        try:
            # stop servoing (if available)
            if hasattr(self, '_servo_tab') and self._servo_tab:
                try:
                    self._servo_tab.stop_servoing()
                except Exception as e:
                    self.node.get_logger().warn(f"Failed to stop servoing: {e}")

            src = 'neura_1'
            tgt = 'neura_2'
            all_names = []
            all_positions = []
            joint_pose_tgt = self.node._get_joint_state(tgt)
            joint_pose_src = self.node._get_joint_state(src)
            if not joint_pose_tgt:
                self.node.get_logger().error(f"Failed to get joint state for {tgt}")
                return
            _, positions_tgt = self._get_joint_positions_rad(joint_pose_tgt) 
            all_names.extend(joint_pose_tgt['joint_names'])
            all_positions.extend(positions_tgt)
            all_names.extend(joint_pose_src['joint_names'])
            positions_tgt[::2] = [-x for x in positions_tgt[::2]]
            all_positions.extend(positions_tgt)
            self.node.get_logger().info(f"Moving {src} to {tgt} pose")
            self.traj_commander.send_ptp_trajectory(all_names, all_positions)

            # monitor execution and restart servoing when done
            if hasattr(self.traj_commander, 'is_done'):
                timer = QTimer(self)
                timer.setInterval(200)
                def _poll():
                    try:
                        if self.traj_commander.is_done():
                            timer.stop()
                            if hasattr(self, '_servo_tab') and self._servo_tab:
                                try:
                                    self._servo_tab.start_servoing()
                                except Exception as e:
                                    self.node.get_logger().warn(f"Failed to start servoing: {e}")
                    except Exception as e:
                        timer.stop()
                        self.node.get_logger().warn(f"Error while monitoring trajectory: {e}")
                timer.timeout.connect(_poll)
                timer.start()

        except Exception as e:
            self.node.get_logger().error(f"move_to_robot2_pose failed: {e}")

    
    def evaluate_error(self):
        """
        Case A (no fetch yet): compare neura_1/flange and neura_2/flange x to nearest box short-edge x.
        Case B (after fetch): error = aimed_pose - spawn_pose (dx, dy, distance in xy).
        Results go to self.error_distance_y (text) and self.error_distance_x (text).
        """
        try:
            # sanity checks
            if not hasattr(self, "spawn_pose") or not hasattr(self, "spawn_dims"):
                self.node.get_logger().warn("[GUI] No spawned object info yet.")
                self.error_distance_y.setText("No spawn")
                self.error_distance_x.setText("")
                return

            # helpers
            def tf_x_of(frame_id: str) -> float:
                tf = self.tf_buffer.lookup_transform("world_link", frame_id, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.2))
                return tf.transform.translation.x

            # box center & x-size
            x_box = float(self.spawn_pose.position.x)
            sx = float(self.spawn_dims[0])  # box size along x
            x_edge_left  = x_box - sx * 0.5
            x_edge_right = x_box + sx * 0.5

            if self.obj_fetched_btn.isChecked() == False:
                # ---------- Case A: only spawned ----------
                # read robot flange x positions
                try:
                    x_e1 = tf_x_of("neura_1/flange")
                except Exception:
                    x_e1 = float("nan")
                try:
                    x_e2 = tf_x_of("neura_2/flange")
                except Exception:
                    x_e2 = float("nan")

                def nearest_edge_err(x_flange: float) -> float:
                    if not (x_flange == x_flange):  # NaN check
                        return float("nan")
                    # error to the closest short edge
                    e_left  = abs(x_flange - x_edge_left)
                    e_right = abs(x_flange - x_edge_right)
                    return min(e_left, e_right)

                e1 = nearest_edge_err(x_e1)
                e2 = nearest_edge_err(x_e2)

                # show per-robot x errors and a summary (max)
                self.error_distance_y.setText(f"e2_x={e2:.4f}")
                self.error_distance_x.setText(f"e1_x={e1:.4f}")

            else:
                # ---------- Case B: fetched pressed ----------
                if not hasattr(self, "aimed_pose"):
                    self.node.get_logger().warn("[GUI] fetched mode selected but no aimed_pose set.")
                    self.error_distance_y.setText("No aimed_pose")
                    self.error_distance_x.setText("")
                    return
                tf_ee1_to_world = self.tf_buffer.lookup_transform("world_link", "neura_1/flange", rclpy.time.Time(), rclpy.duration.Duration(seconds=0.2))
                obj_world_position = tf2_geometry_msgs.do_transform_pose(self.obj_pose_wrt_ee1, tf_ee1_to_world)
                dx = float(self.aimed_pose.position.x) - float(obj_world_position.position.x)
                dy = float(self.aimed_pose.position.y) - float(obj_world_position.position.y)

                self.error_distance_y.setText(f"dy={dy:.4f}")
                self.error_distance_x.setText(f"dx={dx:.4f}")

        except Exception as e:
            self.node.get_logger().error(f"[GUI] evaluate_error failed: {e}")
            self.error_distance_y.setText("error")
            self.error_distance_x.setText("")