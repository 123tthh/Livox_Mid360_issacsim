#!/usr/bin/env python3
"""Install the pinned MID360 robot assets into a Lidar_Hiking checkout."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

from validate_assets import validate


ROOT = Path(__file__).resolve().parents[1]
ASSET_ROOT = ROOT / "assets"
RELATIVE_TARGET = Path("simulation/source/lidar_hiking_sim/lidar_hiking_sim/assets")
ASSET_DIRS = ("g1_29dof_rev_1_0", "g1_29dof_mode_13_5010")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("project_root", type=Path)
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()

    validate()
    target_root = args.project_root.resolve() / RELATIVE_TARGET
    if not target_root.parent.is_dir():
        raise FileNotFoundError(f"not a Lidar_Hiking checkout: {args.project_root}")
    for directory in ASSET_DIRS:
        source = ASSET_ROOT / directory
        target = target_root / directory
        if not args.verify_only:
            shutil.copytree(source, target, dirs_exist_ok=True)
        print(f"{'verified' if args.verify_only else 'installed'}: {target}")


if __name__ == "__main__":
    main()
