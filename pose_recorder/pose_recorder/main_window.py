#!/usr/bin/env python3
import sys
import re
import os
import yaml
import rclpy
import threading  # Import the threading module
from datetime import datetime
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QScrollArea, QHeaderView,
    QVBoxLayout, QHBoxLayout, QFormLayout, QTabWidget, QFrame, QGridLayout,QTableWidget, QTableWidgetItem,
    QLabel, QLineEdit, QComboBox, QPushButton, QGroupBox, QCheckBox, QMessageBox, QFileDialog, QListWidget
)
from PySide6.QtWidgets import QDialog, QVBoxLayout, QHBoxLayout, QListWidget, QPushButton
from PySide6.QtGui import QIcon
from PySide6.QtCore import QTimer, Qt
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory
import subprocess

# Import the existing ROS2 nodes
from pose_recorder.pose_recorder import RobotPoseRecorder
from pose_recorder.trajectory_commander import TrajectoryCommander
from pose_recorder.tabs.pose_editor_tab import PoseEditorTab
from pose_recorder.tabs.tf_settings_tab import TFSettingsTab

from pose_recorder.tabs.space_mouse_tab import SpaceMouseTab
from pose_recorder.tabs.servoing_tab import ServoingTab

logger = rclpy.logging.get_logger("pose_recorder_gui")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Multi-Arm Pose Recorder")
        icon_path = os.path.join(get_package_share_directory('pose_recorder'), 'resource', 'fhglogo.png')
        logger.info(f"Icon path: {icon_path}")
        self.setWindowIcon(QIcon(icon_path))

        rclpy.init()

        self.node = RobotPoseRecorder()

        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        self.ros_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.ros_thread.start()

        self.save_dir_edit = QLineEdit(self.node.save_dir)

        self.traj_commander = TrajectoryCommander(self.node)
        self.tf_settings_tab = TFSettingsTab(self, self.node, self.executor)
        self.pose_editor_tab = PoseEditorTab(self, self.node, self.executor, self.traj_commander)
        self.space_mouse_tab = SpaceMouseTab(self, self.node, self.executor)

        self.servoing_tab = ServoingTab(self, self.node, self.executor)


        # After UI setup, populate the pose list:
        self.create_layout()
        
        self.show()
        self.tf_settings_tab.update_tf_fields()
        self.tf_settings_tab.init_dropdowns()


    def create_layout(self):
        
        # --- UI Elements ---
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)

        # Tab Layout
        tabs = QTabWidget()
        #tabs.setTabPosition(QTabWidget.North)

        # Tab 1: TF Settings
        tab1 = self.tf_settings_tab
        tabs.addTab(tab1, "Settings")

        # Tab 2: Pose Editor
        tab2 = self.pose_editor_tab
        tabs.addTab(tab2, "Poses")


        # Tab 3: Space Mouse Configuration
        tab6 = self.space_mouse_tab
        tabs.addTab(tab6, "Space")

        # Tab 4: Servoing Configuration
        tab7 = self.servoing_tab
        tabs.addTab(tab7, "Servo")


        # Add tabs to main layout
        main_layout.addWidget(tabs)
    

    def handle_pose_renamed(self, old_name, new_name):
        
        # Refresh the pose list widget in Pose Editor
        self.pose_editor_tab.update_pose_list_widget()

        # Refresh pose list in Sequence Editor
        self.sequence_editor_tab._handle_pose_renamed(old_name, new_name)
    

    # Cross-class functions -> handles information that needs to be transferred across several tabs
    def get_robot_list(self):
        robots = []
        for edit in self.tf_settings_tab.robot_name_edits.values():
            if isinstance(edit, QLineEdit):
                txt = edit.text().strip()
                if txt:
                    robots.append(txt)
        return robots
    
    def get_enabled_robots(self):
        return self.tf_settings_tab.get_enabled_robots()
        
    def is_robot_checked(self, idx):
        return self.tf_settings_tab.robot_checkboxes[idx].isChecked()

    def get_save_dir(self):
        return self.tf_settings_tab.get_save_dir()

    def get_sequence_dir(self):
        return self.tf_settings_tab.get_sequences_dir()
    
    def get_relpose_dir(self):
        return self.tf_settings_tab.get_relpose_dir()

    def get_config_dir(self):
        return self.tf_settings_tab.get_config_dir()
    
    def get_servo_active(self):
        return self.servoing_tab.get_servo_active()
    
    def get_tf_filter_params(self):
        tf_whitelist            = self.tf_settings_tab.get_tf_whitelist()
        tf_override_keywords    = self.tf_settings_tab.get_tf_override_keywords()
        return tf_whitelist, tf_override_keywords
    
    def start_servo(self):
        self.servoing_tab.start_servoing()
    
    def stop_servo(self):
        self.servoing_tab.stop_servoing()

    def get_base_frame(self, idx):
        return self.tf_settings_tab.base_frame_cbs[idx].currentText()
    
    def get_ee_frame(self, idx):
        return self.tf_settings_tab.ee_frame_cbs[idx].currentText()

    def send_gripper_command(self, robot, cmd):
        """
        Sends a gripper command to the robot's gripper.
        """
        if cmd == "open":
            self.grippers_tab._send_open(robot)
        elif cmd == "close":
            self.grippers_tab._send_close(robot)
        elif cmd == "move":
            self.grippers_tab._send_move(robot)
        elif cmd == "grasp":
            self.grippers_tab._send_grasp(robot)
    
    def get_available_grippers(self):
        return self.grippers_tab.get_available_grippers()


    def closeEvent(self, event):
        # Clean shutdown of ROS
        rclpy.shutdown()
        if self.ros_thread.is_alive():
            self.ros_thread.join()
        self.node.destroy_node()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    sys.exit(app.exec())
