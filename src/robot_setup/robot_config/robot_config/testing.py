"""Helpers for tests: a complete config built from the template."""
from __future__ import annotations

from typing import Any, Dict

from .config import _merge, template_for


def full_config(entry: str = "whole_body", **sections: Any) -> Dict[str, Any]:
    """The template of an entry with the given sections merged in, e.g.

        full_config(robot={"hardware": "mock_components"}, cameras={"enable": False})

    A dict value is merged into the section, any other value replaces it."""
    config = template_for(entry)
    _merge(config, sections)
    return config
