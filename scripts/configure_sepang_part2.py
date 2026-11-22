#!/usr/bin/env python3
"""Configure Part 2: Sepang with five obstacles and two F1Tenth vehicles.

Preserves the Part 1 setup script. Makes a timestamped backup of sim.yaml.
Requires the edited PNG and its YAML to exist in this repository under maps/.
"""
from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import re
import shutil

ROOT = Path(__file__).resolve().parents[1]
MAPS = ROOT / "maps"
MAP_NAME = "Sepang_5obs_map"
FILES = (MAP_NAME + ".png", MAP_NAME + ".yaml")


def main() -> None:
    parser = argparse.ArgumentParser(description="Configure F1Tenth Part 2 on Sepang.")
    parser.add_argument("--workspace", type=Path, required=True,
                        help="ROS workspace containing src/f1tenth_gym_ros")
    args = parser.parse_args()
    ws = args.workspace.expanduser().resolve()
    config = ws / "src/f1tenth_gym_ros/config/sim.yaml"
    destination = ws / "src/f1tenth_gym_ros/maps"

    if not config.is_file():
        parser.error(f"Simulator sim.yaml not found: {config}")
    missing = [str(MAPS / f) for f in FILES if not (MAPS / f).is_file()]
    if missing:
        parser.error("Missing Part 2 map file(s): " + ", ".join(missing) +
                     ". Copy your definitive five-obstacle PNG into maps/ first.")

    original = config.read_text(encoding="utf-8")
    settings = {
        "map_path": f"'{destination / MAP_NAME}'",
        "map_img_ext": "'.png'",
        "num_agent": "2",
        "sx": "-14.14",
        "sy": "-1.14",
        "stheta": "0.08",
        "sx1": "-9.16",
        "sy1": "-0.74",
        "stheta1": "0.08",
    }
    updated = original
    for key, value in settings.items():
        pattern = rf"(?m)^(\s*{re.escape(key)}\s*:)\s*[^\n]*$"
        if len(re.findall(pattern, updated)) != 1:
            parser.error(f"Expected one {key}: in {config}; no changes made")
        updated = re.sub(pattern,
                         lambda match, replacement=value: match.group(1) + " " + replacement,
                         updated, count=1)

    backup = config.with_name(
        "sim.yaml.backup_part2_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    )
    destination.mkdir(parents=True, exist_ok=True)
    shutil.copy2(config, backup)
    for filename in FILES:
        shutil.copy2(MAPS / filename, destination / filename)
    config.write_text(updated, encoding="utf-8")
    print("Part 2 configured: Sepang + five obstacles + two autonomous cars")
    print(f"Map: {destination / MAP_NAME}")
    print("Ego: (-14.14, -1.14, 0.08); opponent: (-9.16, -0.74, 0.08)")
    print("Check both initial poses in RViz (5 m center-to-center approximation).")
    print(f"Original sim.yaml backup: {backup}")
    print(f"Build: cd {ws} && colcon build --symlink-install")


if __name__ == "__main__":
    main()
