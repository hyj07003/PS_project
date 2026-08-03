from __future__ import annotations

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

    app.extensions["db"] = conn
    app.extensions["services"] = {
        "users": users,
        "products": products,
        "carts": carts,
        "orders": orders,
    }

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
