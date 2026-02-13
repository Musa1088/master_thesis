# pose_recorder

`pose_recorder` is a ROS 2–based graphical user interface (GUI) designed for recording, editing, and replaying robot poses in single- and dual-arm teleoperation setups.  
The tool provides a structured interface for configuring robots, managing poses and motion sequences, integrating 6-DoF input devices, and applying geometric constraints for coordinated manipulation.
Tailored for the dual-arm Neura robot setup, works with the existing moveit setup in `neura_workcell`.
Should be placed and used inside `neura_ws`.

---

## Overview of the GUI

The GUI is organized into multiple tabs, each addressing a specific aspect of teleoperation and motion management:

- **Settings Tab**
- **Poses Tab**
- **Space Mouse Tab**
- **Servo Tab**
- **Geometric Constraints Tab**

Together, these tabs allow users to configure the system, record and edit poses, control robots via a 6-DoF input device, and enforce geometric relationships between multiple manipulators.

---

## Settings Tab

The **Settings Tab** is used for global configuration of the teleoperation system.

Main functionalities include:
- Selection and filtering of TF frames
- Configuration of robot names, base frames, and end-effector frames
- Enabling or disabling individual robots
- Defining directories for storing poses, sequences, relative poses, and configuration files

This tab establishes the foundational system configuration and must be set correctly before pose recording or teleoperation.

---

## Poses Tab

The **Poses Tab** manages recorded robot poses.

Features:
- Display of all available recorded poses
- Creation, editing, and deletion of poses
- Opening a dedicated **Pose Editor** window when a pose is selected
- Editing poses in both joint space and Cartesian space
- Executing a selected pose and visualizing the target in RViz

Poses are stored in YAML format and can be reused across sessions and experiments.

---

## Space Mouse Tab

The **Space Mouse Tab** provides configuration for 6-DoF input devices, such as a SpaceMouse, through the `teleop_twist_joy` node.

Supported functionalities:
- Selection of the teleoperation node
- Mapping of linear and angular axes
- Scaling and turbo factors for translational and rotational motion
- Enabling or disabling individual axes
- Saving, loading, and fetching SpaceMouse configurations as YAML files

This tab allows precise tuning of the teleoperation interface to match user preferences and task requirements.

---

## Servo Tab

The **Servo Tab** exposes parameters for **MoveIt Servo**, enabling real-time Cartesian velocity control.

Key options include:
- Selection of the Servo node
- Scaling of linear and angular velocities
- Collision checking and slowdown thresholds
- Prediction horizon configuration
- Selection of Cartesian input velocity topics for each robot
- Starting and stopping the Servo-based control loop
- Opening the **Geometric Constraints** tab

This tab bridges high-level teleoperation commands with low-level robot motion execution.

---

## Geometric Constraints Tab

The **Geometric Constraints Tab** enables the definition and activation of geometric relationships between multiple robots during teleoperation.

Supported constraints include:
- **Fixed Distance**
- **Mirror Motion**
- **Object-Centered Spherical Rotation**
- **Line Motion**
- **Planar (Surface) Motion**

Additional functionalities:
- User-defined surface normal vectors for planar motion
- Selection of active robots (single or dual operation)
- Choice of feedback control strategy (e.g., Master–Slave or Sensor Velocity Control)
- Rescue functionality to recover from singularities or collision-induced stops
- User study utilities, such as spawning test objects and error evaluation

---

## Launching the Node
```bash
ros2 launch pose_recorder main.launch.py


