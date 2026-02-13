from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QListWidget, QSizePolicy,
    QPushButton, QFileDialog, QMessageBox, QGroupBox, QLabel, QLineEdit, QAbstractItemView
)

import os
import yaml
from pose_recorder.pose_editor import EditPoseWindow
from datetime import datetime
import rclpy

logger = rclpy.logging.get_logger("pose_editor_tab")


class PoseEditorTab(QWidget):
    def __init__(self, parent, node, executor, traj_commander):
        super().__init__(parent)
        self.node = node
        self.executor = executor
        self.parent = parent
        self.traj_commander = traj_commander
        self.all_poses = []  # [{'name': base, 'description': str}]

        self.create_layout()

        # Initial population
        self.update_pose_list_widget()
    
    def create_layout(self):

        # --- UI setup ---
        layout = QVBoxLayout(self)

        search_row = QHBoxLayout()
        self.search_edit = QLineEdit()
        self.search_edit.setPlaceholderText("Search poses by name or description…")
        self.search_edit.textChanged.connect(self.filter_poses)
        clear_btn = QPushButton("Clear")
        clear_btn.clicked.connect(self.search_edit.clear)
        search_row.addWidget(QLabel("Search:"))
        search_row.addWidget(self.search_edit, 1)
        search_row.addWidget(clear_btn)
        layout.addLayout(search_row)

        # List of poses
        pose_grp = QGroupBox("Available Poses")
        pose_grp.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        pose_layout = QVBoxLayout(pose_grp)
        self.pose_list = QListWidget()
        self.pose_list.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.pose_list.itemDoubleClicked.connect(self.open_selected_pose)
        pose_layout.addWidget(self.pose_list)
        layout.addWidget(pose_grp)

        # Buttons
        btn_layout = QHBoxLayout()
        for text, slot in [
            ("Create", self.save_pose_callback),
            ("Edit",     self.edit_pose_callback),
            ("Delete",   self.delete_pose),
            ("Refresh",  self.refresh_poses_callback),
        ]:
            btn = QPushButton(text)
            btn.clicked.connect(slot)
            btn_layout.addWidget(btn)
        layout.addLayout(btn_layout)


    def _scan_pose_dir(self):
        """(Re)build in-memory index of pose names + descriptions."""
        dir_path = self.parent.get_save_dir()
        poses = []
        if os.path.isdir(dir_path):
            for fname in sorted(os.listdir(dir_path)):
                if fname.lower().endswith(('.yaml', '.yml')):
                    path = os.path.join(dir_path, fname)
                    name = os.path.splitext(fname)[0]
                    desc = ""
                    try:
                        with open(path, "r", encoding="utf-8") as f:
                            data = yaml.safe_load(f) or {}

                        # Prefer nested header.description, then top-level fallbacks
                        header = data.get("header") or {}
                        raw_desc = header.get("description")
                        if raw_desc is None:
                            raw_desc = data.get("description", data.get("pose_description", ""))

                        desc = str(raw_desc) if raw_desc is not None else ""
                    except Exception as e:
                        # Keep going; just log
                        logger.warning(f"Failed to read description from {path}: {e}")

                    poses.append({"name": name, "description": desc})
        self.all_poses = poses


    def _render_names(self, names):
        self.pose_list.clear()
        for n in names:
            self.pose_list.addItem(n)


    def filter_poses(self):
        """Filter self.all_poses using search text (case-insensitive)."""
        q = self.search_edit.text().strip().lower()
        if not q:
            self._render_names([p["name"] for p in self.all_poses])
            return

        filtered = []
        for p in self.all_poses:
            name_match = q in p["name"].lower()
            desc_match = q in (p["description"] or "").lower()
            if name_match or desc_match:
                filtered.append(p["name"])

        self._render_names(filtered)
    

    def delete_pose(self):
        items = self.pose_list.selectedItems()
        if not items:
            QMessageBox.information(self, "Delete Pose(s)", "No poses selected to delete.")
            return

        bases = [it.text() for it in items]
        plural = "poses" if len(bases) > 1 else "pose"
        preview = "\n".join(bases[:10]) + ("\n…" if len(bases) > 10 else "")
        resp = QMessageBox.question(
            self,
            "Delete Pose(s)",
            f"Delete {len(bases)} {plural}?\n\n{preview}",
            QMessageBox.Yes | QMessageBox.No
        )
        if resp != QMessageBox.Yes:
            return

        dir_path = self.parent.get_save_dir()
        deleted = 0
        errors = []

        for base in bases:
            file_path = None
            for ext in ('.yaml', '.yml'):
                candidate = os.path.join(dir_path, base + ext)
                if os.path.isfile(candidate):
                    file_path = candidate
                    break
            if not file_path:
                errors.append(f"File for '{base}' not found.")
                continue

            try:
                os.remove(file_path)
                deleted += 1
            except Exception as e:
                errors.append(f"Failed to delete '{base}': {e}")

        self.update_pose_list_widget()

        if errors:
            QMessageBox.warning(
                self, "Delete Pose(s)",
                f"Deleted {deleted} of {len(bases)}.\n\nIssues:\n- " + "\n- ".join(errors)
            )

    
    
    # def delete_pose(self):
    #     item = self.pose_list.currentItem()
    #     if not item:
    #         QMessageBox.information(self, "Delete Pose", "No pose selected to delete.")
    #         return
    #     base = item.text()
    #     # Confirm deletion
    #     resp = QMessageBox.question(
    #         self,
    #         "Delete Pose",
    #         f"Are you sure you want to delete pose '{base}'?",
    #         QMessageBox.Yes | QMessageBox.No
    #     )
    #     if resp != QMessageBox.Yes:
    #         return
    #     # Determine file path (.yaml or .yml)
    #     dir_path = self.parent.get_save_dir()
    #     file_path = None
    #     for ext in ('.yaml', '.yml'):
    #         candidate = os.path.join(dir_path, base + ext)
    #         if os.path.isfile(candidate):
    #             file_path = candidate
    #             break
    #     if not file_path:
    #         QMessageBox.warning(self, "Delete Pose", f"Could not find file for pose '{base}'.")
    #         return
    #     # Attempt removal
    #     try:
    #         os.remove(file_path)
    #         #QMessageBox.information(self, "Delete Pose", f"Pose '{base}' deleted.")
    #     except Exception as e:
    #         QMessageBox.critical(self, "Delete Pose", f"Failed to delete pose '{base}': {e}")
    #         return
    #     # Refresh list
    #     self.update_pose_list_widget()


    # def update_pose_list_widget(self):
    #     dir_path = self.parent.get_save_dir()
    #     self.pose_list.clear()
    #     if os.path.isdir(dir_path):
    #         for fname in sorted(os.listdir(dir_path)):
    #             if fname.lower().endswith(('.yaml', '.yml')):
    #                 base = os.path.splitext(fname)[0]
    #                 self.pose_list.addItem(base)
    #                 #self.pose_list.addItem(fname)

    # --- replace your update_pose_list_widget() with this version ---
    def update_pose_list_widget(self):
        self._scan_pose_dir()
        self.filter_poses()  # renders all if search is empty


    def open_selected_pose(self, item):
        dir_path = self.parent.get_save_dir()
        base = item.text()
        file_path = None
        for ext in ('.yaml', '.yml'):
            candidate = os.path.join(dir_path, base + ext)
            if os.path.isfile(candidate):
                file_path = candidate
                break
        if file_path:
            self.edit_pose_window = EditPoseWindow(file_path, self.node, self.parent, self.traj_commander)
            self.edit_pose_window.pose_renamed.connect(self.parent.handle_pose_renamed)
            self.edit_pose_window.show()
        else:
            QMessageBox.warning(self, "File not found", f"Could not find file for pose: {base}")
        self.update_pose_list_widget()


    def edit_pose_callback(self):
        item = self.pose_list.currentItem()
        if not item:
            QMessageBox.information(self, "Edit Pose", "No pose selected to edit.")
            return
        dir_path = self.parent.get_save_dir()
        base = item.text()
        file_path = None
        for ext in ('.yaml', '.yml'):
            candidate = os.path.join(dir_path, base + ext)
            if os.path.isfile(candidate):
                file_path = candidate
                break
        if file_path:
            self.edit_pose_window = EditPoseWindow(file_path, self.node, self.parent, self.traj_commander)
            self.edit_pose_window.pose_renamed.connect(self.parent.handle_pose_renamed)
            self.edit_pose_window.show()
        else:
            QMessageBox.warning(self, "File not found", f"Could not find file for pose: {base}")
        # Optionally refresh the list if names might change:
        self.update_pose_list_widget()
    
    def handle_pose_renamed(self):
        
        # Refresh the pose list widget
        self.update_pose_list_widget()


    def save_pose_callback(self):
        # Update node parameters from UI
        robots = self.parent.get_robot_list()
        self.node.robot_names = robots
        #self.node.world_frame = self.world_frame_edit.text()
        self.node.save_dir    = self.parent.save_dir_edit.text()
        self.node.pose_name = "__recorded_" + datetime.now().strftime("%d.%m.%Y__%H:%M:%S")
        self.node.pose_description = ""
        robots_checked = {}
        # Update robot config
        for idx, robot in enumerate(self.parent.get_robot_list()):
            base = self.parent.get_base_frame(idx)
            ee   = self.parent.get_ee_frame(idx)
            self.node.robot_config[robot]['base_frame'] = base
            self.node.robot_config[robot]['ee_frame']   = ee
            robots_checked[robot] = self.parent.is_robot_checked(idx) #Hereeeeeee fix this line
        # Check which robots are checked
        checked = [r for r, ok in robots_checked.items() if ok]
        if not checked:
            QMessageBox.critical(
                self,
                "Error",
                "At least one robot must be selected before saving pose."
            )
            return
        which_robot = 0 # equals 1 when only robot1 is checked. Equals 2 when only robot2 is checked. Equals 3 when both of the robots are checked
        for i, robot in enumerate(robots):
            if robots_checked[robot]:
                which_robot |= (1 << i)
        # Trigger the pose lookup and save
        self.pose_data = self.node.lookup_pose(which_robot)
        self.node.save_pose(self.pose_data)

        # After saving, refresh list:
        self.update_pose_list_widget()

        # Open the newly created pose in the editor
        dir_path = self.parent.get_save_dir()
        base_name = self.node.pose_name
        if base_name:
            new_file = os.path.join(dir_path, base_name + '.yaml')
            if os.path.isfile(new_file):
                self.edit_pose_window = EditPoseWindow(new_file, self.node, self.parent, self.traj_commander)
                self.edit_pose_window.pose_renamed.connect(self.parent.handle_pose_renamed)
                self.edit_pose_window.show()
            else:
                QMessageBox.warning(self, "File not found", f"Could not find newly saved pose: {base_name}.yaml")
        
        # After saving, refresh list:
        self.update_pose_list_widget()

    def refresh_poses_callback(self):
        """‘Refresh’ button: re-scan the directory."""
        self.update_pose_list_widget()
