from __future__ import annotations

from flask import Blueprint, current_app, jsonify, request

from .errors import ApiError

bp = Blueprint("api", __name__)


def _truthy(value: str | None) -> bool:
    return value in ("1", "true")


def _users():
    return current_app.extensions["services"]["users"]


def _products():
    return current_app.extensions["services"]["products"]


def _carts():
    return current_app.extensions["services"]["carts"]


def _orders():
    return current_app.extensions["services"]["orders"]


@bp.get("/health")
def health():
    return jsonify({"ok": True, "service": "controller-server"})


@bp.post("/users/register")
def register():
    body = request.get_json(silent=True) or {}
    return jsonify(
        _users().register(body.get("email"), body.get("password"), body.get("name"))
    )


@bp.post("/users/login")
def login():
    body = request.get_json(silent=True) or {}
    return jsonify(_users().verify_login(body.get("email"), body.get("password")))


@bp.get("/users/<int:user_id>")
def get_user(user_id: int):
    return jsonify(_users().find_by_id(user_id))


@bp.get("/categories")
def categories():
    return jsonify(_products().list_categories())


@bp.get("/products")
def list_products():
    return jsonify(
        _products().list(
            q=request.args.get("q"),
            category=request.args.get("category"),
            featured=_truthy(request.args.get("featured")),
            include_inactive=_truthy(request.args.get("includeInactive")),
        )
    )


@bp.get("/products/<int:product_id>")
def get_product(product_id: int):
    return jsonify(
        _products().get_by_id(
            product_id,
            active_only=_truthy(request.args.get("activeOnly")),
        )
    )


@bp.post("/products")
def create_product():
    body = request.get_json(silent=True) or {}
    created_by = body.pop("createdBy", None)
    return jsonify(_products().create(body, created_by))


@bp.put("/products/<int:product_id>")
@bp.patch("/products/<int:product_id>")
def update_product(product_id: int):
    body = request.get_json(silent=True) or {}
    return jsonify(_products().update(product_id, body))


@bp.delete("/products/<int:product_id>")
def delete_product(product_id: int):
    return jsonify(_products().remove(product_id))


@bp.get("/carts/<int:user_id>")
def get_cart(user_id: int):
    return jsonify(_carts().get_cart(user_id))


@bp.post("/carts/<int:user_id>/items")
def add_cart_item(user_id: int):
    body = request.get_json(silent=True) or {}
    return jsonify(
        _carts().add_item(
            user_id,
            body.get("productId"),
            body.get("quantity", 1) or 1,
        )
    )


@bp.patch("/carts/<int:user_id>/items/<int:product_id>")
def update_cart_item(user_id: int, product_id: int):
    body = request.get_json(silent=True) or {}
    return jsonify(_carts().update_item(user_id, product_id, body.get("quantity", 0)))


@bp.delete("/carts/<int:user_id>/items/<int:product_id>")
def remove_cart_item(user_id: int, product_id: int):
    return jsonify(_carts().remove_item(user_id, product_id))


@bp.post("/carts/<int:user_id>/merge")
def merge_cart(user_id: int):
    body = request.get_json(silent=True) or {}
    return jsonify(_carts().merge_guest(user_id, body.get("items") or []))


@bp.post("/orders")
def create_order():
    body = request.get_json(silent=True) or {}
    user_id = body.get("userId")
    if user_id is None:
        raise ApiError(400, "userId is required")
    return jsonify(_orders().create_from_cart(int(user_id)))


@bp.get("/orders/<int:order_id>")
def get_order(order_id: int):
    return jsonify(_orders().get_by_id(order_id))


@bp.get("/devices")
def devices():
    return jsonify(_orders().list_devices())


def _robot():
    return current_app.extensions["services"]["robot"]


@bp.get("/missions")
def list_missions():
    return jsonify(
        _robot().list_missions(
            status=request.args.get("status"),
            device_code=request.args.get("deviceCode"),
        )
    )


@bp.get("/missions/<int:mission_id>")
def get_mission(mission_id: int):
    return jsonify(_robot().get_mission(mission_id))


@bp.patch("/missions/<int:mission_id>")
def patch_mission(mission_id: int):
    body = request.get_json(silent=True) or {}
    status = body.get("status")
    if not status:
        raise ApiError(400, "status is required")
    return jsonify(_robot().patch_mission(mission_id, status, body.get("note")))


@bp.patch("/devices/<code>")
def patch_device(code: str):
    body = request.get_json(silent=True) or {}
    status = body.get("status")
    if not status:
        raise ApiError(400, "status is required")
    return jsonify(_robot().patch_device(code, status))


@bp.post("/robot/telemetry")
def post_telemetry():
    body = request.get_json(silent=True) or {}
    return jsonify(_robot().save_telemetry(body))


@bp.get("/robot/telemetry")
def list_telemetry():
    return jsonify(_robot().list_telemetry())


@bp.get("/robot/telemetry/<code>")
def get_telemetry(code: str):
    data = _robot().get_telemetry(code)
    if data is None:
        raise ApiError(404, "telemetry not found")
    return jsonify(data)
