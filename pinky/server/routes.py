from __future__ import annotations

from flask import Blueprint, current_app, jsonify, request, Response

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
                "POST /nav/initialpose",
                "POST /nav/goal",
                "POST /nav/stop",
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


@bp.get("/sensors")
def sensors_all():
    return jsonify(_robot().snapshot().to_dict())


@bp.get("/sensors/battery")
def sensors_battery():
    return jsonify(_robot().battery.read().to_dict())


@bp.get("/sensors/lidar")
def sensors_lidar():
    return jsonify(_robot().lidar.read().to_dict())


@bp.get("/sensors/imu")
def sensors_imu():
    return jsonify(_robot().imu.read().to_dict())


@bp.get("/sensors/ultrasonic")
def sensors_ultrasonic():
    return jsonify(_robot().ultrasonic.read().to_dict())


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


@bp.post("/nav/initialpose")
def nav_initialpose():
    body = request.get_json(silent=True) or {}
    try:
        x = float(body["x"])
        y = float(body["y"])
    except (KeyError, TypeError, ValueError):
        return jsonify({"success": False, "message": "x,y required"}), 400
    yaw = float(body.get("yaw", 0.0))
    result = _robot().navigation.set_initial_pose(x, y, yaw)
    status = 200 if result.get("success") else 502
    return jsonify(result), status


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


@bp.post("/nav/stop")
def nav_stop():
    result = _robot().navigation.cancel()
    status = 200 if result.get("success") else 502
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
