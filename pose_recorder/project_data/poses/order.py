#!/usr/bin/env python3
import yaml
from pathlib import Path
import sys
import logging

logging.basicConfig(level=logging.INFO, format="%(message)s")
logger = logging.getLogger("fix_joint_order")

DESIRED_ORDER = [f"joint_{i}" for i in range(1, 8)]  # joint_1 ... joint_7

def fix_joint_order_in_file(path: Path) -> None:
    try:
        content = path.read_text()
    except Exception as e:
        logger.warning(f"Skipping {path.name}: cannot read file: {e}")
        return

    try:
        data = yaml.safe_load(content)
    except Exception as e:
        logger.warning(f"Skipping {path.name}: YAML parse error: {e}")
        return

    if not isinstance(data, dict):
        logger.debug(f"{path.name}: top-level is not a dict, skipping")
        return

    changed = False

    for robot_key in list(data.keys()):
        if not robot_key.startswith("robot_"):
            continue
        robot_section = data.get(robot_key, {})
        joint_pose = robot_section.get("joint_pose", {})
        jad = joint_pose.get("joint_angles_degrees")
        if isinstance(jad, dict):
            # Rebuild ordered dict: desired order first, then any extras
            new_jad = {}
            for name in DESIRED_ORDER:
                if name in jad:
                    new_jad[name] = jad[name]
            for name in jad:
                if name not in DESIRED_ORDER:
                    new_jad[name] = jad[name]
            if list(jad.keys()) != list(new_jad.keys()):
                data[robot_key]["joint_pose"]["joint_angles_degrees"] = new_jad
                changed = True

    if changed:
        try:
            # Write back with stable ordering (don't sort other keys)
            with path.open("w") as f:
                yaml.safe_dump(data, f, sort_keys=False)
            logger.info(f"Reordered joints in {path.name}")
        except Exception as e:
            logger.error(f"Failed to write updated file {path.name}: {e}")
    else:
        logger.debug(f"No changes needed for {path.name}")


def main():
    script_dir = Path(__file__).resolve().parent
    patterns = ["*.yaml", "*.yml"]
    files = []
    for patt in patterns:
        files.extend(script_dir.glob(patt))
    if not files:
        logger.info("No YAML files found to process.")
        return

    for f in sorted(files):
        fix_joint_order_in_file(f)

if __name__ == "__main__":
    main()
