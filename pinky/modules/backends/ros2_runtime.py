from __future__ import annotations

import threading
from typing import Any

_lock = threading.Lock()
_rclpy = None
_executor = None
_spin_thread: threading.Thread | None = None
_started = False
_nodes: list[Any] = []


def ensure_runtime() -> Any:
    """Initialize rclpy once and return the module."""
    global _rclpy, _executor, _spin_thread, _started
    with _lock:
        if _rclpy is None:
            import rclpy
            from rclpy.executors import MultiThreadedExecutor

            _rclpy = rclpy
            if not rclpy.ok():
                rclpy.init(args=None)
            _executor = MultiThreadedExecutor(num_threads=4)
        if not _started:
            _started = True
            _spin_thread = threading.Thread(target=_spin_loop, daemon=True, name="ros2-spin")
            _spin_thread.start()
        return _rclpy


def add_node(node: Any) -> None:
    ensure_runtime()
    with _lock:
        assert _executor is not None
        _executor.add_node(node)
        _nodes.append(node)


def remove_node(node: Any) -> None:
    with _lock:
        if _executor is None:
            return
        try:
            _executor.remove_node(node)
        except Exception:
            pass
        if node in _nodes:
            _nodes.remove(node)
        try:
            node.destroy_node()
        except Exception:
            pass


def _spin_loop() -> None:
    assert _rclpy is not None and _executor is not None
    while _started and _rclpy.ok():
        try:
            _executor.spin_once(timeout_sec=0.1)
        except Exception:
            break


def shutdown() -> None:
    global _started, _executor, _rclpy, _spin_thread
    with _lock:
        _started = False
        for node in list(_nodes):
            try:
                if _executor:
                    _executor.remove_node(node)
                node.destroy_node()
            except Exception:
                pass
        _nodes.clear()
        if _rclpy is not None:
            try:
                _rclpy.shutdown()
            except Exception:
                pass
        _executor = None
        _rclpy = None
        _spin_thread = None
