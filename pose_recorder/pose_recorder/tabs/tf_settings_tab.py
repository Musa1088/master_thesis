from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QListWidget, QLineEdit, QLabel,
    QPushButton, QFileDialog, QMessageBox, QGroupBox, QFormLayout, QComboBox, QLayout,
    QCheckBox, QTreeWidget, QTreeWidgetItem, QDialog, QGraphicsScene, QGraphicsView, QSizePolicy
)
from PySide6.QtGui import QPen, QBrush, QColor, QFont, QPainter
from PySide6.QtCore import QPointF, Qt
import os
import yaml
from datetime import datetime
from pose_recorder.utils import get_tf_frames
import rclpy
import re

logger = rclpy.logging.get_logger("tf_settings_tab")

class TFSettingsTab(QWidget):
    def __init__(self, parent, node, executor):
        super().__init__(parent)
        self.node = node
        self.executor = executor
        self.parent = parent

        # Class variables
        self.base_frame_cbs = {}
        self.ee_frame_cbs = {}
        self.robot_groups = []
        self.robot_name_edits = {}
        self.robot_checkboxes = {}

        self.tf_frames_initialized = False
        self.save_pose_dir = self.node.save_dir
        self.save_sequence_dir = self.node.sequence_dir
        self.save_relpose_dir = self.node.relpose_dir
        self.save_config_dir = self.node.config_dir
        self.tf_whitelist = None
        self.tf_override_keywords = []

        self.create_layout()


    def create_layout(self):
        layout = QVBoxLayout(self)

        # --- in create_layout(), before the per-robot panels, insert filter UI ---
        filter_box = QGroupBox("TF Frame Filtering")
        filter_box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        filter_form = QFormLayout(filter_box)
        self.filter_whitelist_le = QLineEdit()
        self.filter_whitelist_le.setPlaceholderText("e.g. world, flange")
        self.filter_whitelist_le.setText("world, flange")  # default whitelist
        self.override_keywords_le = QLineEdit()
        self.override_keywords_le.setPlaceholderText("e.g. world_link")
        self.override_keywords_le.setText("world_link")  # default override keyword
        self.filter_enable_cb = QCheckBox("Enable filtering")
        self.filter_enable_cb.setChecked(True)  # default to enabled
        filter_form.addRow("Whitelist:", self.filter_whitelist_le)
        filter_form.addRow("Globals:", self.override_keywords_le)
        filter_form.addRow("", self.filter_enable_cb)
        layout.addWidget(filter_box)

        # connect to trigger updates
        self.filter_enable_cb.toggled.connect(self.update_tf_fields)
        #self.filter_whitelist_le.textChanged.connect(lambda _: self.update_tf_fields() if self.filter_enable_cb.isChecked() else None)
        #self.override_keywords_le.textChanged.connect(self.update_tf_fields)

        # 1) One panel per robot
        for i, _ in enumerate(self.node.robot_names):
            layout.addWidget(self._make_robot_panel(i))

        # 2) Save‐pose section (world-frame & save-dir)
        layout.addWidget(self._make_save_dir_group())

        # 3) Bottom buttons
        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self._make_button("Refresh TF Frames", self.update_tf_fields))
        btn_layout.addWidget(self._make_button("Controller", self.open_controller_window))
        btn_layout.addWidget(self._make_button("TF Tree", self.show_tf_tree_dialog))
        layout.addLayout(btn_layout)


    def _make_robot_panel(self, idx: int) -> QGroupBox:

        name_edit = QLineEdit(f"{self.node.robot_names[idx]}")
        self.robot_name_edits[idx] = name_edit

        form = QFormLayout()
        form.addRow("Name:", name_edit)
        base_cb = QComboBox()
        ee_cb = QComboBox()
        form.addRow("Base frame:", base_cb)
        form.addRow("EE frame:",   ee_cb)
        #v.addLayout(form)

        chk = QCheckBox()
        chk.setChecked(True)
        form.addRow("Enable:", chk)

        # wire up enable toggle
        chk.toggled.connect(lambda checked, idx=idx: self._on_robot_enable_toggled(idx, checked))

        # initialize enabled/disabled state according to the checkbox
        self.robot_checkboxes[idx] = chk
        self.base_frame_cbs[idx] = base_cb
        self.ee_frame_cbs[idx]   = ee_cb
        self._on_robot_enable_toggled(idx, chk.isChecked())

        # wrap in group box
        box = QGroupBox(f"Robot {idx+1}")
        box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        v = QVBoxLayout(box)
        v.addLayout(form)

        return box


    def _make_save_dir_group(self) -> QGroupBox:

        form = QFormLayout()

        self.save_dir_edit = QLineEdit(self.node.save_dir)
        browse = QPushButton("Browse…")
        browse.clicked.connect(self.browse_save_directory)
        h = QHBoxLayout(); h.addWidget(self.save_dir_edit); h.addWidget(browse)
        form.addRow("Poses:", h)

        self.seq_dir_edit = QLineEdit(self.node.sequence_dir)
        browse = QPushButton("Browse…")
        browse.clicked.connect(self.browse_sequence_directory)
        h = QHBoxLayout(); h.addWidget(self.seq_dir_edit); h.addWidget(browse)
        form.addRow("Sequences:", h)

        self.relpose_dir_edit = QLineEdit(self.node.relpose_dir)
        browse = QPushButton("Browse…")
        browse.clicked.connect(self.browse_relpose_directory)
        h = QHBoxLayout(); h.addWidget(self.relpose_dir_edit); h.addWidget(browse)
        form.addRow("Relative:", h)
    
        self.config_dir_edit = QLineEdit(self.node.config_dir)
        browse = QPushButton("Browse…")
        browse.clicked.connect(self.browse_config_directory)
        h = QHBoxLayout(); h.addWidget(self.config_dir_edit); h.addWidget(browse)
        form.addRow("Configs:", h)

        # wrap in group box
        box = QGroupBox("Directories")
        #box.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        v = QVBoxLayout(box)
        v.addLayout(form)
        return box


    def _make_button(self, text: str, slot) -> QPushButton:
        btn = QPushButton(text)
        btn.clicked.connect(slot)
        return btn

    def browse_save_directory(self):
        dir_path = QFileDialog.getExistingDirectory(self, "Select Save Directory", self.save_dir_edit.text())
        if dir_path:
            self.save_dir_edit.setText(dir_path)
            self.save_pose_dir = dir_path
            #self.update_pose_list_widget()
    
    def browse_sequence_directory(self):
        dir_path = QFileDialog.getExistingDirectory(self, "Select Sequence Directory", self.seq_dir_edit.text())
        if dir_path:
            self.seq_dir_edit.setText(dir_path)
            self.save_sequence_dir = dir_path
    
    def browse_relpose_directory(self):
        dir_path = QFileDialog.getExistingDirectory(self, "Select Sequence Directory", self.relpose_dir_edit.text())
        if dir_path:
            self.relpose_dir_edit.setText(dir_path)
            self.save_relpose_dir = dir_path
    
    def browse_config_directory(self):
        dir_path = QFileDialog.getExistingDirectory(self, "Select Config Directory", self.config_dir_edit.text())
        if dir_path:
            self.config_dir_edit.setText(dir_path)
            self.save_config_dir = dir_path
    
    def get_save_dir(self):
        return self.save_pose_dir

    def get_sequences_dir(self):
        return self.save_sequence_dir

    def get_relpose_dir(self):
        return self.save_relpose_dir

    def get_config_dir(self):
        return self.save_config_dir
    
    def get_enabled_robots(self) -> list[str]:
        """
        Return the robot names corresponding to checked "Enable" boxes.
        """
        robots = self.parent.get_robot_list()
        return [
            robots[idx]
            for idx, cb in self.robot_checkboxes.items()
            if idx < len(robots) and cb.isChecked()
        ]
    
    def _on_robot_enable_toggled(self, idx: int, enabled: bool):
        # enable/disable the name edit and frame selectors based on the checkbox
        name_edit = self.robot_name_edits.get(idx)
        if name_edit:
            name_edit.setEnabled(enabled)
        base_cb = self.base_frame_cbs.get(idx)
        ee_cb = self.ee_frame_cbs.get(idx)
        if base_cb:
            base_cb.setEnabled(enabled)
        if ee_cb:
            ee_cb.setEnabled(enabled)
    
    
    def get_tf_whitelist(self):
        if getattr(self, "filter_enable_cb", None) and self.filter_enable_cb.isChecked():
            raw = self.filter_whitelist_le.text()
            return [w.strip() for w in re.split(r'[;,]', raw) if w.strip()]
        return None
    
    def get_tf_override_keywords(self):
        if getattr(self, "filter_enable_cb", None) and self.filter_enable_cb.isChecked():
            raw_overrides = getattr(self, "override_keywords_le", None) and self.override_keywords_le.text() or ""
            if raw_overrides.strip():
                return [w.strip() for w in re.split(r'[;,]', raw_overrides) if w.strip()]
        return None
    

    def update_tf_fields(self):
        # Query all TF frames from the buffer
        robots = self.parent.get_robot_list()
        logger.info(f"Robots: {robots}")

        tf_whitelist = self.get_tf_whitelist()
        tf_override_keywords = self.get_tf_override_keywords()

        num_slots = len(self.base_frame_cbs)
        for idx in range(num_slots):
            base_cb = self.base_frame_cbs.get(idx)
            ee_cb = self.ee_frame_cbs.get(idx)
            if idx >= len(robots):
                for cb in (base_cb, ee_cb):
                    if cb:
                        cb.clear()
                continue

            name = robots[idx]
            prevs = {
                'base': base_cb.currentText() if base_cb else None,
                'ee': ee_cb.currentText() if ee_cb else None,
            }
            logger.info(f"Updating TF frames for robot {name} (base: {prevs['base']}, ee: {prevs['ee']})")

            frames = get_tf_frames(
                self.node,
                [name],
                whitelist=tf_whitelist,
                override_keywords=tf_override_keywords,
            )

            for key, combo in (('base', base_cb), ('ee', ee_cb)):
                if not combo:
                    continue
                prev = prevs[key]
                combo.clear()
                combo.addItems(frames)
                if prev:
                    match_index = combo.findText(prev)
                    if match_index >= 0:
                        combo.setCurrentIndex(match_index)
                    elif combo.count():
                        combo.setCurrentIndex(0)
                elif combo.count():
                    combo.setCurrentIndex(0)

        # if tf_frames are updated for the first time
        if not getattr(self, "tf_frames_initialized", False):
            self.init_dropdowns()
            self.tf_frames_initialized = True



    def init_dropdowns(self):
        """
        After update_tf_fields has populated the QComboBoxes,
        set their current values based on self.node.base_frames and self.node.ee_frames.
        """
        robots = self.parent.get_robot_list()
        base_defaults = getattr(self.node, "base_frames", [])
        ee_defaults = getattr(self.node, "ee_frames", [])

        def apply_default(combo, default_value):
            if combo is None:
                return
            if default_value:
                match_index = combo.findText(default_value)
                if match_index >= 0:
                    combo.setCurrentIndex(match_index)
                elif combo.count() > 0:
                    combo.setCurrentIndex(0)
            else:
                if combo.count() > 0:
                    combo.setCurrentIndex(0)

        for idx in range(len(robots)):
            base_combo = self.base_frame_cbs.get(idx)
            ee_combo = self.ee_frame_cbs.get(idx)

            base_default = base_defaults[idx] if idx < len(base_defaults) else None
            ee_default = ee_defaults[idx] if idx < len(ee_defaults) else None

            apply_default(base_combo, base_default)
            apply_default(ee_combo, ee_default)



    def open_controller_window(self):   
        self.controller_window = ControllerGUI(self.node)
        self.controller_window.show()


    def show_tf_tree_dialog(self):
        """
        Pop up a dialog that draws each TF frame and a line
        directly to its parent (no other metadata or extraneous nodes).
        """
        # 1) Fetch and parse the YAML dump
        try:
            yaml_str = self.node.tf_buffer.all_frames_as_yaml()
            raw: dict = yaml.safe_load(yaml_str) or {}
        except Exception as e:
            QMessageBox.warning(self, "TF Error", f"Could not load TF tree:\n{e}")
            return

        # 2) Build parent_map directly from the flat dict
        parent_map = {}
        for frame, props in raw.items():
            parent = props.get("parent")
            if parent:
                parent_map[frame] = parent

        # 3) Compute a simple layout: group by “depth” from world_link
        depths = {}
        def compute_depth(f):
            if f in depths:
                return depths[f]
            p = parent_map.get(f)
            if not p or p == f:
                depths[f] = 0
            else:
                depths[f] = compute_depth(p) + 1
            return depths[f]

        for f in set(parent_map) | set(parent_map.values()):
            compute_depth(f)

        by_depth = {}
        for f, d in depths.items():
            by_depth.setdefault(d, []).append(f)

        # 4) Create the Qt dialog + scene
        dlg = QDialog(self)
        dlg.setWindowTitle("TF Parent–Child Graph")
        dlg.resize(800, 600)

        vlay = QVBoxLayout(dlg)
        scene = QGraphicsScene()
        view = QGraphicsView(scene)
        # enable antialiasing
        view.setRenderHint(QPainter.Antialiasing, True)
        vlay.addWidget(view)

        # zoom and close controls
        hlay = QHBoxLayout()
        zin   = QPushButton("+")
        zout  = QPushButton("–")
        close = QPushButton("Close")
        for w in (zin, zout):
            hlay.addWidget(w)
        hlay.addStretch()
        hlay.addWidget(close)
        vlay.addLayout(hlay)

        zin.clicked.connect(lambda: view.scale(1.2, 1.2))
        zout.clicked.connect(lambda: view.scale(1/1.2, 1/1.2))
        close.clicked.connect(dlg.accept)

        # 5) Place nodes in the scene
        NODE_SPACING_X = 200
        LEVEL_SPACING_Y = 120
        text_items = {}

        for depth, frames in sorted(by_depth.items()):
            count = len(frames)
            for idx, f in enumerate(sorted(frames)):
                x = (idx - (count-1)/2) * NODE_SPACING_X
                y = depth * LEVEL_SPACING_Y
                ti = scene.addText(f, QFont("Arial", 10))
                ti.setDefaultTextColor(Qt.darkBlue)
                bb = ti.boundingRect()
                # center it
                ti.setPos(x - bb.width()/2, y - bb.height()/2)
                text_items[f] = ti

        # 6) Draw one line per child→parent
        pen = QPen(Qt.black)
        pen.setWidth(2)
        for child, parent in parent_map.items():
            cti = text_items.get(child)
            pti = text_items.get(parent)
            if not cti or not pti:
                continue
            # from bottom-center of parent to top-center of child
            p_bb = pti.boundingRect()
            c_bb = cti.boundingRect()
            p_pt = pti.pos() + QPointF(p_bb.width()/2, p_bb.height())
            c_pt = cti.pos() + QPointF(c_bb.width()/2, 0)
            scene.addLine(p_pt.x(), p_pt.y(), c_pt.x(), c_pt.y(), pen)

        dlg.exec()