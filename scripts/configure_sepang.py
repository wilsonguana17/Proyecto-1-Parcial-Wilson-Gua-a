#!/usr/bin/env python3
"""Configure the Sepang map in an existing F1TENTH ROS 2 simulator workspace.

No controller logic is modified. This script copies the two map files and
updates only six keys in src/f1tenth_gym_ros/config/sim.yaml after backing it up.
"""
from __future__ import annotations
import argparse
from datetime import datetime
from pathlib import Path
import re
import shutil

REPO_ROOT = Path(__file__).resolve().parents[1]
MAP_DIR = REPO_ROOT / 'maps'
MAP_FILES = ('Sepang_map.png', 'Sepang_map.yaml')


def main() -> None:
    parser = argparse.ArgumentParser(description='Configure Sepang in a F1TENTH ROS 2 workspace')
    parser.add_argument('--workspace', type=Path, required=True,
                        help='ROS 2 workspace containing src/f1tenth_gym_ros')
    args = parser.parse_args()
    workspace = args.workspace.expanduser().resolve()
    config = workspace / 'src/f1tenth_gym_ros/config/sim.yaml'
    map_dest = workspace / 'src/f1tenth_gym_ros/maps'
    if not config.is_file():
        parser.error(f'Missing simulator configuration: {config}')
    for filename in MAP_FILES:
        if not (MAP_DIR / filename).is_file():
            parser.error(f'Missing map file: {MAP_DIR / filename}')

    original = config.read_text(encoding='utf-8')
    values = {
        'map_path': "'" + str(map_dest / 'Sepang_map') + "'",
        'map_img_ext': "'.png'",
        'num_agent': '1',
        'sx': '-14.14',
        'sy': '-1.14',
        'stheta': '0.08',
    }
    updated = original
    for key, value in values.items():
        # Only a YAML key at the beginning of a line, not sx1/stheta1.
        pattern = rf'(?m)^(\s*{re.escape(key)}\s*:)\s*[^\n]*$'
        if len(list(re.finditer(pattern, updated))) != 1:
            parser.error(f'Expected exactly one {key}: line in {config}; no files changed')
        updated = re.sub(pattern, lambda match, value=value: match.group(1) + ' ' + value,
                         updated, count=1)

    map_dest.mkdir(parents=True, exist_ok=True)
    backup = config.with_name('sim.yaml.backup_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    shutil.copy2(config, backup)
    for filename in MAP_FILES:
        shutil.copy2(MAP_DIR / filename, map_dest / filename)
    config.write_text(updated, encoding='utf-8')
    print('Sepang configuration completed.')
    print(f'Workspace: {workspace}')
    print(f'Map prefix: {map_dest / "Sepang_map"}')
    print(f'Config backup: {backup}')
    print('Start pose: x=-14.14 m, y=-1.14 m, heading=0.08 rad')
    print(f'Next: cd "{workspace}" && colcon build --symlink-install')


if __name__ == '__main__':
    main()
