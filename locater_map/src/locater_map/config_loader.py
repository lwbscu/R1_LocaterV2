from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "default_config.json"


def deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def load_config(path: str | Path | None = None) -> dict[str, Any]:
    with DEFAULT_CONFIG.open("r", encoding="utf-8") as f:
        config = json.load(f)
    if path:
        with Path(path).open("r", encoding="utf-8") as f:
            config = deep_merge(config, json.load(f))
    config["_project_root"] = str(PROJECT_ROOT)
    apply_prior_map_config(config)
    return config


def apply_prior_map_config(config: dict[str, Any]) -> None:
    map_cfg = config.get("map", {})
    prior_path = resolve_resource(config, map_cfg.get("prior_map_config"))
    if not prior_path or not prior_path.exists():
        return
    with prior_path.open("r", encoding="utf-8") as f:
        prior = json.load(f)
    for src_key, dst_key in (
        ("field_width_cm", "field_width_cm"),
        ("field_height_cm", "field_height_cm"),
        ("pixels_per_cm_x", "pixels_per_cm_x"),
        ("pixels_per_cm_y", "pixels_per_cm_y"),
        ("image_width_px", "image_width_px"),
        ("image_height_px", "image_height_px"),
    ):
        if src_key in prior:
            map_cfg[dst_key] = prior[src_key]


def resolve_resource(config: dict[str, Any], relative_path: str | None) -> Path | None:
    if not relative_path:
        return None
    path = Path(relative_path)
    if path.is_absolute():
        return path
    return Path(config.get("_project_root", PROJECT_ROOT)) / path
