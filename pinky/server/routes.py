from __future__ import annotations

from flask import Blueprint, current_app, jsonify, request, Response
from typing import Any

bp = Blueprint("api", __name__)


def _robot():
    return current_app.extensions["robot"]


def _controller():
    return current_app.extensions["controller"]


def _device_code() -> str:
    return current_app.extensions["device_code"]


@bp.get("/")
def index():
    robot = _robot()
    return jsonify(
        {
            "ok": True,
            "service": "pinky-server",
            "message": "Pinky robot API. Use /health or /sensors.",
            "backend": robot.backend_name,
            "deviceCode": _device_code(),
            "online": robot.snapshot().online,
            "endpoints": [
                "GET /health",
                "GET /publisher/status",
                "GET /sensors",
                "GET /sensors/battery",
                "GET /sensors/lidar",
                "GET /sensors/imu",
                "GET /sensors/ultrasonic",
                "GET /map/meta",
                "GET /map/image",
                "GET /nav/state",
                "GET /nav/plan",
                "GET /nav/path",
                "POST /nav/plan",
                "POST /nav/initialpose",
                "POST /nav/goal",
                "POST /nav/goal_wait",
                "POST /nav/stop",
                "POST /nav/relative_move",
                "POST /nav/aruco_dock",
                "POST /nav/aruco_undock",
                "POST /actuators/led",
                "POST /actuators/lcd",
            ],
        }
    )


@bp.get("/publisher/status")
def publisher_status():
    pub = current_app.extensions.get("sensor_publisher")
    if pub is None:
        return jsonify({"running": False, "hardware": []})
    return jsonify(
        {
            "running": True,
            "hardware": pub.hardware_status,
        }
    )


@bp.get("/health")
def health():
    robot = _robot()
    pub = current_app.extensions.get("sensor_publisher")
    nav = robot.navigation.state()
    return jsonify(
        {
            "ok": True,
            "service": "pinky-server",
            "backend": robot.backend_name,
            "deviceCode": _device_code(),
            "online": robot.snapshot().online,
            "sensorPublisher": bool(pub),
            "mapId": nav.get("mapId"),
            "navigating": nav.get("navigating"),
        }
    )


def _annotate_sensor_sources(payload: dict) -> dict:
    """Overlay publisher fallback sources + isDummy flags onto sensor dicts."""
    from modules.types import is_dummy_source

    pub = current_app.extensions.get("sensor_publisher")
    sources = {}
    if pub is not None and hasattr(pub, "sensor_sources"):
        sources = pub.sensor_sources
    robot = _robot()
    if robot.backend_name == "mock":
        for key in ("battery", "imu", "ultrasonic", "lidar"):
            sources.setdefault(key, "mock")

    for key, src in sources.items():
        block = payload.get(key)
        if not isinstance(block, dict):
            continue
        # publisher dummy/fallback overrides topic-level "ros2"
        if src and src != "unknown":
            block["source"] = src
        block["isDummy"] = is_dummy_source(block.get("source"))
        if block["isDummy"] and key == "battery":
            # keep explicit label for UI
            block["dummyLabel"] = "더미값"
    return payload


@bp.get("/sensors")
def sensors_all():
    return jsonify(_annotate_sensor_sources(_robot().snapshot().to_dict()))


@bp.get("/sensors/battery")
def sensors_battery():
    data = _robot().battery.read().to_dict()
    return jsonify(_annotate_sensor_sources({"battery": data})["battery"])


@bp.get("/sensors/lidar")
def sensors_lidar():
    data = _robot().lidar.read().to_dict()
    return jsonify(_annotate_sensor_sources({"lidar": data})["lidar"])


@bp.get("/sensors/imu")
def sensors_imu():
    data = _robot().imu.read().to_dict()
    return jsonify(_annotate_sensor_sources({"imu": data})["imu"])


@bp.get("/sensors/ultrasonic")
def sensors_ultrasonic():
    data = _robot().ultrasonic.read().to_dict()
    return jsonify(_annotate_sensor_sources({"ultrasonic": data})["ultrasonic"])


# ----- Map + Navigation -----


@bp.get("/map/meta")
def map_meta():
    info = _robot().navigation.map_info()
    if not info:
        return jsonify({"error": "map not found (PINKY_MAP / map_test1.yaml)"}), 404
    return jsonify(info)


@bp.get("/map/image")
def map_image():
    try:
        png = _robot().navigation.map_png()
    except FileNotFoundError as exc:
        return jsonify({"error": str(exc)}), 404
    except RuntimeError as exc:
        return jsonify({"error": str(exc)}), 500
    return Response(png, mimetype="image/png")


@bp.get("/nav/state")
def nav_state():
    return jsonify(_robot().navigation.state())


@bp.get("/nav/path")
def nav_path():
    """Compat alias for tools expecting /nav/path (same cache as GET /nav/plan)."""
    path = _robot().navigation.get_path() or {
        "frameId": "map",
        "count": 0,
        "poses": [],
    }
    poses = path.get("poses") if isinstance(path, dict) else None
    ok = bool(poses)
    return jsonify(
        {
            "success": ok,
            "message": None if ok else "no path received yet",
            "path": path,
        }
    )


@bp.get("/nav/plan")
def nav_plan_get():
    """Nav2 /plan (global path) snapshot for remote clients."""
    return jsonify(_robot().navigation.get_plan())


@bp.post("/nav/plan")
def nav_plan_compute():
    """Compute a Nav2 global path without starting robot motion."""
    body = request.get_json(silent=True) or {}
    try:
        x = float(body["x"])
        y = float(body["y"])
    except (KeyError, TypeError, ValueError):
        return jsonify({"success": False, "message": "x,y required"}), 400
    yaw = float(body.get("yaw", 0.0))
    timeout = float(body.get("timeoutSec", body.get("timeout_sec", 10.0)))
    planner_id = str(body.get("plannerId", body.get("planner_id", "")))
    result = _robot().navigation.plan_to(x, y, yaw, timeout, planner_id)
    status = 200 if result.get("success") else 502
    return jsonify(result), status


@bp.post("/nav/initialpose")
def nav_initialpose():
    body = request.get_json(silent=True) or {}
    try:
        x = float(body["x"])
        y = float(body["y"])
    except (KeyError, TypeError, ValueError):
        return jsonify({"success": False, "message": "x,y required"}), 400
    yaw = float(body.get("yaw", 0.0))
    try:
        result = _robot().navigation.set_initial_pose(x, y, yaw)
    except Exception as exc:
        current_app.logger.exception("POST /nav/initialpose")
        return jsonify({"success": False, "message": str(exc)}), 502
    if not result.get("success") and not result.get("ignored"):
        current_app.logger.warning(
            "POST /nav/initialpose failed: %s", result.get("message")
        )
        return jsonify(result), 502
    return jsonify(result), 200


@bp.post("/nav/goal")
def nav_goal():
    body = request.get_json(silent=True) or {}
    try:
        x = float(body["x"])
        y = float(body["y"])
    except (KeyError, TypeError, ValueError):
        return jsonify({"success": False, "message": "x,y required"}), 400
    yaw = float(body.get("yaw", 0.0))
    result = _robot().navigation.go_to(x, y, yaw)
    status = 200 if result.get("success") else 502
    return jsonify(result), status


@bp.post("/nav/goal_wait")
def nav_goal_wait():
    """Send NavigateToPose and block until arrived / failed / timeout."""
    body = request.get_json(silent=True) or {}
    try:
        x = float(body["x"])
        y = float(body["y"])
    except (KeyError, TypeError, ValueError):
        return jsonify({"success": False, "message": "x,y required"}), 400
    yaw = float(body.get("yaw", 0.0))
    timeout = float(body.get("timeoutSec", body.get("timeout_sec", 180.0)))
    result = _robot().navigation.go_to_wait(x, y, yaw, timeout)
    status = 200 if result.get("success") else 502
    return jsonify(result), status


@bp.post("/nav/stop")
def nav_stop():
    body = request.get_json(silent=True) or {}
    freeze_raw = body.get("freeze", True)
    freeze = str(freeze_raw).strip().lower() not in ("0", "false", "no", "off")
    result = _robot().navigation.cancel(freeze=freeze)
    status = 200 if result.get("success") else 502
    return jsonify(result), status


@bp.post("/nav/relative_move")
def nav_relative_move():
    """Short odom-closed-loop micro motion for docking/undocking only."""
    body = request.get_json(silent=True) or {}
    try:
        distance_m = float(body.get("distanceM", body.get("distance_m")))
    except (TypeError, ValueError):
        return jsonify({"success": False, "message": "distanceM required"}), 400
    try:
        speed_mps = float(body.get("speedMps", body.get("speed_mps", 0.02)))
        timeout_raw = body.get("timeoutSec", body.get("timeout_sec"))
        timeout_sec = float(timeout_raw) if timeout_raw is not None else None
    except (TypeError, ValueError):
        return jsonify({"success": False, "message": "invalid speedMps/timeoutSec"}), 400
    dry_run = bool(body.get("dryRun", body.get("dry_run", False)))
    bypass_collision = bool(
        body.get("bypassCollision", body.get("bypass_collision", False))
    )
    ignore_scan = bool(body.get("ignoreScan", body.get("ignore_scan", False)))
    result = _robot().navigation.relative_move(
        distance_m,
        speed_mps,
        timeout_sec,
        dry_run=dry_run,
        bypass_collision=bypass_collision,
        ignore_scan=ignore_scan,
    )
    status = 200 if result.get("success") else 409
    return jsonify(result), status


@bp.post("/nav/aruco_dock")
def nav_aruco_dock():
    """Visual servo to ArUco marker (~standoffM via pose / open-loop)."""
    body = request.get_json(silent=True) or {}
    try:
        marker_id = int(body.get("markerId", body.get("marker_id")))
    except (TypeError, ValueError):
        return jsonify({"success": False, "status": "FAILED", "message": "markerId required"}), 400
    standoff = body.get("standoffM", body.get("standoff_m"))
    timeout = body.get("timeoutSec", body.get("timeout_sec"))
    try:
        standoff_m = float(standoff) if standoff is not None else None
    except (TypeError, ValueError):
        return jsonify({"success": False, "status": "FAILED", "message": "invalid standoffM"}), 400
    try:
        timeout_sec = float(timeout) if timeout is not None else None
    except (TypeError, ValueError):
        return jsonify({"success": False, "status": "FAILED", "message": "invalid timeoutSec"}), 400
    try:
        result = _robot().navigation.aruco_dock(
            marker_id,
            standoff_m=standoff_m,
            timeout_sec=timeout_sec,
        )
    except Exception as exc:
        return jsonify(
            {
                "success": False,
                "status": "FAILED",
                "message": f"aruco_dock exception: {exc}",
            }
        ), 502
    status = 200 if result.get("success") else 502
    return jsonify(result), status


@bp.post("/nav/aruco_undock")
def nav_aruco_undock():
    """Reverse until marker/ultrasonic range reaches pre-approach target."""
    body = request.get_json(silent=True) or {}
    try:
        marker_id = int(body.get("markerId", body.get("marker_id")))
    except (TypeError, ValueError):
        return jsonify({"success": False, "message": "markerId required"}), 400
    try:
        target_range_m = float(
            body.get("targetRangeM", body.get("target_range_m"))
        )
    except (TypeError, ValueError):
        return jsonify({"success": False, "message": "targetRangeM required"}), 400
    timeout = body.get("timeoutSec", body.get("timeout_sec"))
    speed = body.get("speedMps", body.get("speed_mps"))
    max_travel = body.get("maxTravelM", body.get("max_travel_m"))
    kwargs: dict[str, Any] = {}
    try:
        if timeout is not None:
            kwargs["timeout_sec"] = float(timeout)
        if speed is not None:
            kwargs["speed_mps"] = float(speed)
        if max_travel is not None:
            kwargs["max_travel_m"] = float(max_travel)
    except (TypeError, ValueError):
        return jsonify({"success": False, "message": "invalid undock params"}), 400
    try:
        result = _robot().navigation.aruco_undock(
            marker_id,
            target_range_m,
            **kwargs,
        )
    except Exception as exc:
        return jsonify(
            {
                "success": False,
                "status": "FAILED",
                "message": f"aruco_undock exception: {exc}",
            }
        ), 502
    status = 200 if result.get("success") else 409
    return jsonify(result), status


@bp.post("/actuators/led")
def actuators_led():
    body = request.get_json(silent=True) or {}
    command = body.get("command", "fill")
    r = int(body.get("r", 0))
    g = int(body.get("g", 0))
    b = int(body.get("b", 0))
    pixels = body.get("pixels") or []
    robot = _robot()
    if command == "clear":
        result = robot.led.clear()
    elif command == "set_pixel":
        result = robot.led.set_pixel(list(pixels), r, g, b)
    elif command == "brightness":
        result = robot.led.set_brightness(int(body.get("brightness", 128)))
    else:
        result = robot.led.fill(r, g, b)
    return jsonify(result)


@bp.post("/actuators/lcd")
def actuators_lcd():
    body = request.get_json(silent=True) or {}
    emotion = body.get("emotion", "basic")
    return jsonify(_robot().lcd.set_emotion(emotion))


@bp.get("/actuators/lcd/emotions")
def lcd_emotions():
    return jsonify({"emotions": _robot().lcd.list_emotions()})


@bp.post("/cmd/drive")
def cmd_drive():
    body = request.get_json(silent=True) or {}
    return jsonify(
        _robot().drive(
            float(body.get("linearX", body.get("linear_x", 0.0))),
            float(body.get("angularZ", body.get("angular_z", 0.0))),
        )
    )


@bp.post("/cmd/navigate")
def cmd_navigate():
    """Controller → Pinky: 웨이포인트 이동 명령 (Mock/ROS cmd_vel 기반 간이 시뮬레이션)."""
    body = request.get_json(silent=True) or {}
    waypoint = body.get("waypoint", "aisle-a")
    mission_id = body.get("missionId")
    robot = _robot()
    robot.lcd.set_emotion("interest")
    robot.led.fill(0, 128, 255)
    drive = robot.drive(0.15, 0.0)
    return jsonify(
        {
            "ok": True,
            "status": "ARRIVED",
            "waypoint": waypoint,
            "missionId": mission_id,
            "drive": drive,
        }
    )


@bp.post("/cmd/assign")
def cmd_assign():
    body = request.get_json(silent=True) or {}
    robot = _robot()
    try:
        robot.navigation.prepare_new_job()
    except Exception:
        pass
    robot.lcd.set_emotion("hello")
    robot.led.fill(0, 255, 0)
    return jsonify(
        {
            "ok": True,
            "deviceCode": _device_code(),
            "orderId": body.get("orderId"),
            "missionId": body.get("missionId"),
        }
    )


# ----- Controller API proxy / bridge -----


@bp.get("/controller/health")
def controller_health():
    try:
        return jsonify({"ok": True, "controller": _controller().health()})
    except RuntimeError as exc:
        return jsonify({"ok": False, "error": str(exc)}), 502


@bp.get("/controller/devices")
def controller_devices():
    try:
        return jsonify(_controller().list_devices())
    except RuntimeError as exc:
        return jsonify({"error": str(exc)}), 502


@bp.get("/controller/orders/<int:order_id>")
def controller_order(order_id: int):
    try:
        return jsonify(_controller().get_order(order_id))
    except RuntimeError as exc:
        return jsonify({"error": str(exc)}), 502


@bp.get("/controller/missions")
def controller_missions():
    try:
        return jsonify(
            _controller().list_missions(
                status=request.args.get("status"),
                device_code=request.args.get("deviceCode"),
            )
        )
    except RuntimeError as exc:
        return jsonify({"error": str(exc)}), 502


@bp.get("/controller/missions/<int:mission_id>")
def controller_mission(mission_id: int):
    try:
        return jsonify(_controller().get_mission(mission_id))
    except RuntimeError as exc:
        return jsonify({"error": str(exc)}), 502


@bp.patch("/controller/missions/<int:mission_id>")
def controller_patch_mission(mission_id: int):
    body = request.get_json(silent=True) or {}
    status = body.get("status")
    if not status:
        return jsonify({"error": "status required"}), 400
    try:
        return jsonify(
            _controller().patch_mission(mission_id, status, body.get("note"))
        )
    except RuntimeError as exc:
        return jsonify({"error": str(exc)}), 502


@bp.post("/telemetry/push")
def telemetry_push():
    """현재 센서 스냅샷을 Controller `/robot/telemetry`로 전송."""
    snap = _robot().snapshot().to_dict()
    try:
        result = _controller().post_telemetry(snap)
        return jsonify({"ok": True, "controller": result, "snapshot": snap})
    except RuntimeError as exc:
        return jsonify({"ok": False, "error": str(exc), "snapshot": snap}), 502


@bp.post("/heartbeat")
def heartbeat():
    body = request.get_json(silent=True) or {}
    status = body.get("status", "idle")
    try:
        result = _controller().heartbeat(_device_code(), status)
        return jsonify({"ok": True, "device": result})
    except RuntimeError as exc:
        return jsonify({"ok": False, "error": str(exc)}), 502
