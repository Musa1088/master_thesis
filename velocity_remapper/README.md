# velocity_remapper

`velocity_remapper` is a ROS 2 node responsible for **Cartesian velocity remapping and geometric constraint enforcement** in dual-robot teleoperation setups.  
It takes a single Cartesian twist command and computes synchronized velocity commands for multiple robot end-effectors while respecting user-defined geometric relationships.

The node is designed for **single-controller dual-arm teleoperation**, where one operator input drives coordinated motion of two manipulators.

---

## Purpose and Role in the System

In dual-robot teleoperation, directly sending identical Cartesian velocity commands to both robots leads to drift, loss of relative alignment, or constraint violations.  
The `velocity_remapper` node resolves this by:

- Interpreting a single incoming Cartesian twist command
- Computing appropriate per-robot Cartesian velocity commands
- Enforcing geometric constraints between robot end-effectors
- Publishing constraint-consistent velocity commands for each robot

The node acts as an intermediate layer between **input processing** (e.g., SpaceMouse via `teleop_twist_joy`) and **low-level Cartesian control** (MoveIt Servo).

---

## Supported Geometric Constraints

The following geometric constraint modes are implemented:

- **Fixed Distance**  
  Preserves the initial distance between end-effectors during teleoperation.

- **Mirror Motion**  
  Mirrors the motion of one end-effector with respect to a virtual symmetry plane defined between the robots.

- **Line Motion**  
  Restricts both end-effectors to move only along the line connecting them.

- **Planar (Fixed Surface) Motion**  
  Constrains motion to a plane defined by a user-specified surface normal.

- **Object-Centered Spherical Rotation**  
  Enables rotation of a virtual object held between the robots by commanding circular end-effector motion around a shared center.


---

## Input Topics

The node subscribes to:

- **Cartesian Twist Input**  
  Typically from a 6-DoF input device:

## Launching the Node

Normally, it starts by pressing the `Start Controller Node` in the **Geometric Contraints Tab**. However, one can start the node by 

```bash
ros2 launch velocity_remapper velocity_remapper.launch.py
