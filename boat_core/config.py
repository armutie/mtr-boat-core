from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def load_boat_config(path: str | None) -> dict[str, Any]:
    if not path:
        return {}
    config_path = Path(path)
    if not config_path.exists() and config_path.as_posix() == "config/boat.local.json":
        config_path = Path("config/boat.example.json")
    with config_path.open("r", encoding="utf-8") as file:
        return json.load(file)


def section(config: dict[str, Any], name: str) -> dict[str, Any]:
    value = config.get(name, {})
    return value if isinstance(value, dict) else {}


def choose(cli_value: Any, config: dict[str, Any], key: str, default: Any = None) -> Any:
    return cli_value if cli_value is not None else config.get(key, default)


def choose_bool(cli_value: bool, config: dict[str, Any], key: str, default: bool = False) -> bool:
    return cli_value or bool(config.get(key, default))
