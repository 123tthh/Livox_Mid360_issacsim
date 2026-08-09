#!/usr/bin/env python3
"""Validate published G1+MID360 files against the repository catalog."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / "assets/catalog.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate() -> list[dict[str, str]]:
    catalog = json.loads(CATALOG.read_text(encoding="utf-8"))
    if catalog.get("schema_version") not in (1, 2):
        raise ValueError("unsupported catalog schema")
    results = []
    for name, entry in sorted(catalog["assets"].items()):
        path = ROOT / entry["path"]
        if not path.is_file():
            raise FileNotFoundError(path)
        actual = sha256(path)
        if actual != entry["sha256"]:
            raise ValueError(f"{name}: sha256 {actual} != {entry['sha256']}")
        results.append({"asset": name, "path": entry["path"], "sha256": actual})
    for name, entry in sorted(catalog.get("common_files", {}).items()):
        path = ROOT / entry["path"]
        actual = sha256(path)
        if actual != entry["sha256"]:
            raise ValueError(f"common {name}: sha256 {actual} != {entry['sha256']}")
    return results


if __name__ == "__main__":
    print(json.dumps({"validated": validate()}, indent=2, sort_keys=True))
