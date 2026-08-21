from __future__ import annotations

import sqlite3
from typing import Any

from ..constants import PRODUCT_MAX_STOCK
from ..db import map_product, now_iso
from ..errors import ApiError
from .products import ProductsService


class CartsService:
    def __init__(self, conn: sqlite3.Connection, products: ProductsService):
        self.conn = conn
        self.products = products

    def _ensure_cart(self, user_id: int) -> int:
        existing = self.conn.execute(
            "SELECT id FROM carts WHERE user_id = ?",
            (user_id,),
        ).fetchone()
        if existing:
            return existing["id"]
        ts = now_iso()
        cur = self.conn.execute(
            "INSERT INTO carts (user_id, updated_at) VALUES (?, ?)",
            (user_id, ts),
        )
        self.conn.commit()
        return cur.lastrowid  # type: ignore[return-value]

    def _touch(self, cart_id: int) -> None:
        self.conn.execute(
            "UPDATE carts SET updated_at = ? WHERE id = ?",
            (now_iso(), cart_id),
        )
        self.conn.commit()

    def _product_stock(self, product_id: int) -> int:
        product = self.products.get_by_id(product_id, active_only=True)
        return int(product.get("stock") or 0)

    def _assert_qty_within_stock(self, product_id: int, quantity: int) -> None:
        if quantity < 1:
            raise ApiError(400, "quantity must be >= 1")
        stock = self._product_stock(product_id)
        cap = min(stock, PRODUCT_MAX_STOCK)
        if quantity > cap:
            raise ApiError(
                400,
                f"quantity exceeds available stock ({cap})",
            )

    def get_cart(self, user_id: int) -> dict[str, Any]:
        cart_id = self._ensure_cart(user_id)
        cart = self.conn.execute(
            "SELECT id, user_id, updated_at FROM carts WHERE id = ?",
            (cart_id,),
        ).fetchone()
        items = self.conn.execute(
            """
            SELECT ci.id, ci.product_id, ci.quantity,
                   p.*, c.code AS category_code, c.name AS category_name
            FROM cart_items ci
            JOIN products p ON p.id = ci.product_id
            JOIN categories c ON c.id = p.category_id
            WHERE ci.cart_id = ?
            """,
            (cart_id,),
        ).fetchall()

        return {
            "id": cart["id"],
            "userId": cart["user_id"],
            "updatedAt": cart["updated_at"],
            "items": [
                {
                    "id": row["id"],
                    "productId": row["product_id"],
                    "quantity": row["quantity"],
                    "product": map_product(dict(row)),
                }
                for row in items
            ],
        }

    def add_item(
        self, user_id: int, product_id: int, quantity: int = 1
    ) -> dict[str, Any]:
        if quantity < 1:
            raise ApiError(400, "quantity must be >= 1")
        self.products.get_by_id(product_id, active_only=True)
        cart_id = self._ensure_cart(user_id)
        existing = self.conn.execute(
            "SELECT id, quantity FROM cart_items WHERE cart_id = ? AND product_id = ?",
            (cart_id, product_id),
        ).fetchone()

        new_qty = (existing["quantity"] + quantity) if existing else quantity
        self._assert_qty_within_stock(product_id, new_qty)

        if existing:
            self.conn.execute(
                "UPDATE cart_items SET quantity = ? WHERE id = ?",
                (new_qty, existing["id"]),
            )
        else:
            self.conn.execute(
                "INSERT INTO cart_items (cart_id, product_id, quantity) VALUES (?, ?, ?)",
                (cart_id, product_id, quantity),
            )
        self._touch(cart_id)
        return self.get_cart(user_id)

    def update_item(
        self, user_id: int, product_id: int, quantity: int
    ) -> dict[str, Any]:
        cart_id = self._ensure_cart(user_id)
        if quantity <= 0:
            self.conn.execute(
                "DELETE FROM cart_items WHERE cart_id = ? AND product_id = ?",
                (cart_id, product_id),
            )
        else:
            self._assert_qty_within_stock(product_id, quantity)
            cur = self.conn.execute(
                "UPDATE cart_items SET quantity = ? WHERE cart_id = ? AND product_id = ?",
                (quantity, cart_id, product_id),
            )
            if cur.rowcount == 0:
                raise ApiError(404, "cart item not found")
        self._touch(cart_id)
        return self.get_cart(user_id)

    def remove_item(self, user_id: int, product_id: int) -> dict[str, Any]:
        return self.update_item(user_id, product_id, 0)

    def merge_guest(
        self, user_id: int, items: list[dict[str, Any]]
    ) -> dict[str, Any]:
        for item in items:
            product_id = item.get("productId")
            quantity = item.get("quantity")
            if not product_id or not quantity:
                continue
            try:
                self.add_item(user_id, int(product_id), int(quantity))
            except ApiError:
                pass
        return self.get_cart(user_id)

    def clear(self, user_id: int) -> None:
        cart_id = self._ensure_cart(user_id)
        self.conn.execute("DELETE FROM cart_items WHERE cart_id = ?", (cart_id,))
        self._touch(cart_id)
