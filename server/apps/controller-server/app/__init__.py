from __future__ import annotations

import threading
import time

from flask import Flask, jsonify
from flask_cors import CORS

from .config import load_env
from .db import migrate, open_database
from .errors import ApiError
from .routes import bp
from .seed import seed_if_empty
from .services.carts import CartsService
from .services.orders import OrdersService
from .services.products import ProductsService
from .services.robot import RobotService
from .services.users import UsersService


def create_app() -> Flask:
    load_env()
    app = Flask(__name__)
    CORS(app, origins="*")

    conn = open_database()
    migrate(conn)
    seed_if_empty(conn)

    products = ProductsService(conn)
    users = UsersService(conn)
    carts = CartsService(conn, products)
    orders = OrdersService(conn, carts)
    robot = RobotService(conn, orders)
    robot.set_orders_service(orders)

    app.extensions["db"] = conn
    app.extensions["services"] = {
        "users": users,
        "products": products,
        "carts": carts,
        "orders": orders,
        "robot": robot,
    }

    # Resume queue after restart
    try:
        orders.reclaim_stale_carts()
        orders.try_dispatch()
    except Exception:
        pass

    # cart-1→S1, cart-2→S2 — URL 키 기준 (로봇 DEVICE_CODE 오설정 교정)
    def _home_pose_bootstrap() -> None:
        # 초반 자주, 이후 idle 유지용으로 가끔 재적용
        schedule = [2.0, 6.0, 12.0, 20.0, 35.0, 60.0, 90.0, 120.0]
        elapsed = 0.0
        for delay in schedule:
            time.sleep(max(0.1, delay - elapsed))
            elapsed = delay
            try:
                orders.sync_device_home_poses(only_idle=True)
            except Exception:
                pass
        while True:
            time.sleep(45.0)
            try:
                orders.sync_device_home_poses(only_idle=True)
            except Exception:
                pass

    threading.Thread(
        target=_home_pose_bootstrap, name="home-pose-sync", daemon=True
    ).start()

    @app.errorhandler(ApiError)
    def handle_api_error(err: ApiError):
        return (
            jsonify(
                {
                    "statusCode": err.status_code,
                    "message": err.message,
                    "error": err.error,
                }
            ),
            err.status_code,
        )

    @app.errorhandler(404)
    def handle_not_found(_err):
        return (
            jsonify(
                {
                    "statusCode": 404,
                    "message": "Not Found",
                    "error": "Not Found",
                }
            ),
            404,
        )

    app.register_blueprint(bp)
    return app
