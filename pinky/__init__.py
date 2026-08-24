from __future__ import annotations

"""Compatibility: prefer ``from server import create_app`` (see server/__init__.py)."""

from server import create_app

__all__ = ["create_app"]
