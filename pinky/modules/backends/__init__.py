from .base import RobotBackend
from .mock import MockBackend

__all__ = ["RobotBackend", "MockBackend", "create_backend"]


def create_backend(name: str | None = None) -> RobotBackend:
    backend = (name or "mock").lower().strip()
    if backend == "ros2":
        from .ros2 import Ros2Backend

        return Ros2Backend()
    return MockBackend()
