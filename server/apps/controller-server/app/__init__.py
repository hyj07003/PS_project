from __future__ import annotations

import logging
import os
import threading
import time

from flask import Flask, jsonify
from flask_cors import CORS

from .adapters import OmxHttpStationAdapter, parse_omx_url
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

logger = logging.getLogger(__name__)


def create_app() -> Flask:
    load_env()
    app = Flask(__name__)
    CORS(app, origins="*")

    conn = open_database()
    migrate(conn)
    seed_if_empty(conn)

    products = ProductsService(conn)
    # Demo shelf restock: every controller restart resets all product stock to 3.
    reset = products.reset_all_stock()
    logger.info(
        "product stock reset on startup: stock=%s updated=%s",
        reset.get("stock"),
        reset.get("updated"),
    )
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

    omx_url = parse_omx_url()
    adapter_mode = os.environ.get("ADAPTER_MODE", "").strip().lower()
    if omx_url and adapter_mode != "mock":
        logger.info("OMX remote station configured: %s", omx_url)

        def _omx_startup_check() -> None:
            time.sleep(1.0)
            port = orders.station_port
            if isinstance(port, OmxHttpStationAdapter):
                reachable = port.is_reachable()
                if reachable:
                    logger.info("OMX health OK at %s", omx_url)
                else:
                    logger.warning(
                        "OMX not reachable at %s — picks will use "
                        "unreachable-override until server is up",
                        omx_url,
                    )

        threading.Thread(target=_omx_startup_check, daemon=True).start()
    elif omx_url:
        logger.info(
            "OMX_URL=%s but ADAPTER_MODE=mock — using MockStationAdapter",
            omx_url,
        )
    else:
        logger.info("OMX_URL not set — using MockStationAdapter for shelf picks")

    # Resume queue after restart
    try:
        orders.reclaim_stale_carts()
        orders.try_dispatch()
    except Exception:
        pass

    # cart-1→S1, cart-2→S2 — URL 키 기준 (로봇 DEVICE_CODE 오설정 교정)
    def _home_pose_bootstrap() -> None:
        # 초반 소수회만, 이후 idle 유지용으로 드물게 (AMCL idle freeze 깨우기 최소화)
        schedule = [5.0, 25.0, 90.0]
        elapsed = 0.0
        for delay in schedule:
            time.sleep(max(0.1, delay - elapsed))
            elapsed = delay
            try:
                orders.sync_device_home_poses(only_idle=True)
            except Exception:
                pass
        while True:
            time.sleep(120.0)
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
