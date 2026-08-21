from __future__ import annotations

import sqlite3
import time
from typing import Any

from ..constants import PRODUCT_MAX_STOCK
from ..db import map_product, now_iso, resolve_product_images, slugify
from ..errors import ApiError

PRODUCT_SELECT = """
  SELECT p.*, c.code AS category_code, c.name AS category_name
  FROM products p
  JOIN categories c ON c.id = p.category_id
"""


def clamp_stock(value: Any, default: int = PRODUCT_MAX_STOCK) -> int:
    try:
        stock = int(value if value is not None else default)
    except (TypeError, ValueError):
        stock = default
    return max(0, min(PRODUCT_MAX_STOCK, stock))


class ProductsService:
    def __init__(self, conn: sqlite3.Connection):
        self.conn = conn

    def list_categories(self) -> list[dict[str, Any]]:
        rows = self.conn.execute(
            """
            SELECT id, code, name, sort_order, is_active FROM categories
            WHERE is_active = 1 ORDER BY sort_order ASC
            """
        ).fetchall()
        return [
            {
                "id": r["id"],
                "code": r["code"],
                "name": r["name"],
                "sortOrder": r["sort_order"],
                "isActive": bool(r["is_active"]),
            }
            for r in rows
        ]

    def list(
        self,
        *,
        q: str | None = None,
        category: str | None = None,
        featured: bool = False,
        include_inactive: bool = False,
    ) -> list[dict[str, Any]]:
        where: list[str] = []
        args: list[Any] = []

        if not include_inactive:
            where.append("p.is_active = 1")
        if featured:
            where.append("p.is_featured = 1")
        if category:
            where.append("(c.code = ? OR c.name = ?)")
            args.extend([category, category])
        if q:
            where.append("(p.name LIKE ? OR IFNULL(p.description, '') LIKE ?)")
            like = f"%{q}%"
            args.extend([like, like])

        sql = f"{PRODUCT_SELECT}"
        if where:
            sql += f" WHERE {' AND '.join(where)}"
        sql += " ORDER BY p.is_featured DESC, p.id DESC"

        rows = self.conn.execute(sql, args).fetchall()
        return [map_product(dict(r)) for r in rows]

    def get_by_id(self, product_id: int, active_only: bool = False) -> dict[str, Any]:
        sql = f"{PRODUCT_SELECT} WHERE p.id = ?"
        if active_only:
            sql += " AND p.is_active = 1"
        row = self.conn.execute(sql, (product_id,)).fetchone()
        if not row:
            raise ApiError(404, "product not found")
        return map_product(dict(row))

    def create(self, input_data: dict[str, Any], created_by: int | None = None) -> dict[str, Any]:
        name = input_data.get("name")
        price = input_data.get("price")
        category_id = input_data.get("categoryId")
        if not name or price is None or not category_id:
            raise ApiError(400, "name, price, categoryId required")

        category = self.conn.execute(
            "SELECT id FROM categories WHERE id = ?",
            (category_id,),
        ).fetchone()
        if not category:
            raise ApiError(400, "invalid categoryId")

        ts = now_iso()
        slug = (input_data.get("slug") or "").strip() or slugify(name)
        exists = self.conn.execute(
            "SELECT id FROM products WHERE slug = ?",
            (slug,),
        ).fetchone()
        if exists:
            slug = f"{slug}-{int(time.time() * 1000)}"

        full, zoom = resolve_product_images(
            input_data.get("imageFullUrl"),
            input_data.get("imageZoomUrl"),
        )

        cur = self.conn.execute(
            """
            INSERT INTO products (
              category_id, name, slug, description, price, stock,
              image_full_url, image_zoom_url, is_featured, is_active,
              created_by, created_at, updated_at
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                category_id,
                name,
                slug,
                input_data.get("description"),
                price,
                clamp_stock(input_data.get("stock", PRODUCT_MAX_STOCK)),
                full,
                zoom,
                1 if input_data.get("isFeatured") else 0,
                0 if input_data.get("isActive") is False else 1,
                created_by,
                ts,
                ts,
            ),
        )
        self.conn.commit()
        return self.get_by_id(cur.lastrowid)  # type: ignore[arg-type]

    def update(self, product_id: int, input_data: dict[str, Any]) -> dict[str, Any]:
        current = self.get_by_id(product_id)
        ts = now_iso()
        slug = (input_data.get("slug") or "").strip()
        if not slug:
            slug = slugify(input_data["name"]) if input_data.get("name") else current["slug"]

        next_full = (
            input_data["imageFullUrl"]
            if "imageFullUrl" in input_data
            else current["imageFullUrl"]
        )
        next_zoom = (
            input_data["imageZoomUrl"]
            if "imageZoomUrl" in input_data
            else current["imageZoomUrl"]
        )
        full, zoom = resolve_product_images(next_full, next_zoom)

        if "isFeatured" in input_data:
            is_featured = 1 if input_data["isFeatured"] else 0
        else:
            is_featured = 1 if current["isFeatured"] else 0

        if "isActive" in input_data:
            is_active = 1 if input_data["isActive"] else 0
        else:
            is_active = 1 if current["isActive"] else 0

        description = (
            input_data["description"]
            if "description" in input_data
            else current["description"]
        )

        self.conn.execute(
            """
            UPDATE products SET
              category_id = ?,
              name = ?,
              slug = ?,
              description = ?,
              price = ?,
              stock = ?,
              image_full_url = ?,
              image_zoom_url = ?,
              is_featured = ?,
              is_active = ?,
              updated_at = ?
            WHERE id = ?
            """,
            (
                input_data.get("categoryId", current["categoryId"]),
                input_data.get("name", current["name"]),
                slug,
                description,
                input_data.get("price", current["price"]),
                clamp_stock(
                    input_data.get("stock", current["stock"]),
                    default=int(current["stock"] or 0),
                ),
                full,
                zoom,
                is_featured,
                is_active,
                ts,
                product_id,
            ),
        )
        self.conn.commit()
        return self.get_by_id(product_id)

    def remove(self, product_id: int) -> dict[str, bool]:
        self.get_by_id(product_id)
        order_refs = self.conn.execute(
            "SELECT COUNT(*) AS n FROM order_items WHERE product_id = ?",
            (product_id,),
        ).fetchone()["n"]
        if order_refs:
            raise ApiError(
                409,
                "주문 내역이 있는 상품은 삭제할 수 없습니다. 비활성화해 주세요.",
            )
        self.conn.execute(
            "DELETE FROM cart_items WHERE product_id = ?",
            (product_id,),
        )
        self.conn.execute("DELETE FROM products WHERE id = ?", (product_id,))
        self.conn.commit()
        return {"ok": True}

    def reset_all_stock(self, stock: int = PRODUCT_MAX_STOCK) -> dict[str, Any]:
        """Set every product's stock to the demo shelf capacity (default 3)."""
        value = clamp_stock(stock, default=PRODUCT_MAX_STOCK)
        ts = now_iso()
        cur = self.conn.execute(
            "UPDATE products SET stock = ?, updated_at = ?",
            (value, ts),
        )
        self.conn.commit()
        return {"ok": True, "stock": value, "updated": int(cur.rowcount)}
