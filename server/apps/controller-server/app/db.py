from __future__ import annotations

import re
import sqlite3
import threading
from pathlib import Path
from typing import Any
from urllib.parse import quote, urlparse

_db_lock = threading.RLock()


class LockedConnection:
    """Serialize SQLite access across Flask request threads and mock pipeline."""

    def __init__(self, conn: sqlite3.Connection):
        self._conn = conn

    def execute(self, *args, **kwargs):
        with _db_lock:
            return self._conn.execute(*args, **kwargs)

    def executescript(self, *args, **kwargs):
        with _db_lock:
            return self._conn.executescript(*args, **kwargs)

    def commit(self):
        with _db_lock:
            return self._conn.commit()

    def rollback(self):
        with _db_lock:
            return self._conn.rollback()

    def close(self):
        with _db_lock:
            return self._conn.close()

    def transaction(self):
        return _Transaction(self._conn)


class _Transaction:
    def __init__(self, conn: sqlite3.Connection):
        self._conn = conn

    def __enter__(self):
        _db_lock.acquire()
        self._conn.execute("BEGIN")
        return self._conn

    def __exit__(self, exc_type, exc, tb):
        try:
            if exc_type is None:
                self._conn.commit()
            else:
                self._conn.rollback()
        finally:
            _db_lock.release()
        return False


def now_iso() -> str:
    from datetime import datetime, timezone

    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def to_stored_media_url(url: str | None) -> str | None:
    if not url or not url.strip():
        return None
    trimmed = url.strip()
    if trimmed.startswith("/"):
        return trimmed
    try:
        parsed = urlparse(trimmed)
        if parsed.hostname in ("127.0.0.1", "localhost"):
            return f"{parsed.path}{('?' + parsed.query) if parsed.query else ''}"
    except Exception:
        pass
    return trimmed


def _should_reuse_full_as_zoom(full: str | None, zoom: str | None) -> bool:
    if not full:
        return False
    if not zoom:
        return True
    if full == zoom:
        return False
    if full.startswith("/uploads/") and re.search(r"placehold\.co", zoom, re.I):
        return True
    if (
        full.startswith("/uploads/")
        and re.match(r"^https?://", zoom, re.I)
        and "/uploads/" not in zoom
    ):
        return True
    return False


def resolve_product_images(
    full: str | None,
    zoom: str | None,
) -> tuple[str | None, str | None]:
    image_full = to_stored_media_url(full)
    image_zoom = to_stored_media_url(zoom)

    if _should_reuse_full_as_zoom(image_full, image_zoom):
        image_zoom = image_full
    elif image_zoom and not image_full:
        image_full = image_zoom

    return image_full, image_zoom


def slugify(input_str: str) -> str:
    s = input_str.lower().strip()
    s = re.sub(r"[^\w\uac00-\ud7a3]+", "-", s, flags=re.UNICODE)
    s = re.sub(r"^-+|-+$", "", s)
    s = s[:80]
    if not s:
        import time

        return f"item-{int(time.time() * 1000)}"
    return s


def map_product(row: dict[str, Any]) -> dict[str, Any]:
    full, zoom = resolve_product_images(
        row.get("image_full_url"),
        row.get("image_zoom_url"),
    )
    return {
        "id": row["id"],
        "categoryId": row["category_id"],
        "categoryCode": row.get("category_code"),
        "categoryName": row.get("category_name"),
        "name": row["name"],
        "slug": row["slug"],
        "description": row["description"],
        "price": row["price"],
        "stock": row["stock"],
        "imageFullUrl": full,
        "imageZoomUrl": zoom,
        "isFeatured": bool(row["is_featured"]),
        "isActive": bool(row["is_active"]),
        "createdBy": row["created_by"],
        "createdAt": row["created_at"],
        "updatedAt": row["updated_at"],
    }


def map_user(row: dict[str, Any]) -> dict[str, Any]:
    return {
        "id": row["id"],
        "email": row["email"],
        "name": row["name"],
        "role": row["role"],
        "status": row["status"],
        "createdAt": row["created_at"],
    }


def placeholder(label: str, variant: str) -> str:
    bg = "d8d4cc" if variant == "full" else "c4bfb5"
    fg = "2c2c2c"
    text = quote(label, safe="")
    return f"https://placehold.co/800x1000/{bg}/{fg}/png?text={text}"


def get_database_path() -> Path:
    from .config import get_database_path as _get

    return _get()


def open_database() -> LockedConnection:
    db_path = get_database_path()
    db_path.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(str(db_path), check_same_thread=False)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA journal_mode = WAL")
    conn.execute("PRAGMA foreign_keys = ON")
    return LockedConnection(conn)


def migrate(conn: LockedConnection) -> None:
    conn.executescript(
        """
    CREATE TABLE IF NOT EXISTS users (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      email TEXT NOT NULL UNIQUE,
      password_hash TEXT NOT NULL,
      name TEXT NOT NULL,
      role TEXT NOT NULL CHECK (role IN ('customer', 'admin')),
      status TEXT NOT NULL DEFAULT 'active' CHECK (status IN ('active', 'disabled')),
      created_at TEXT NOT NULL,
      updated_at TEXT NOT NULL
    );
    CREATE INDEX IF NOT EXISTS idx_users_role ON users(role);

    CREATE TABLE IF NOT EXISTS categories (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      code TEXT NOT NULL UNIQUE,
      name TEXT NOT NULL,
      sort_order INTEGER NOT NULL DEFAULT 0,
      is_active INTEGER NOT NULL DEFAULT 1
    );

    CREATE TABLE IF NOT EXISTS products (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      category_id INTEGER NOT NULL REFERENCES categories(id),
      name TEXT NOT NULL,
      slug TEXT NOT NULL UNIQUE,
      description TEXT,
      price INTEGER NOT NULL,
      stock INTEGER NOT NULL DEFAULT 0,
      image_full_url TEXT,
      image_zoom_url TEXT,
      is_featured INTEGER NOT NULL DEFAULT 0,
      is_active INTEGER NOT NULL DEFAULT 1,
      created_by INTEGER REFERENCES users(id),
      created_at TEXT NOT NULL,
      updated_at TEXT NOT NULL
    );
    CREATE INDEX IF NOT EXISTS idx_products_category ON products(category_id);
    CREATE INDEX IF NOT EXISTS idx_products_featured ON products(is_featured);
    CREATE INDEX IF NOT EXISTS idx_products_active ON products(is_active);

    CREATE TABLE IF NOT EXISTS carts (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      user_id INTEGER NOT NULL UNIQUE REFERENCES users(id) ON DELETE CASCADE,
      updated_at TEXT NOT NULL
    );

    CREATE TABLE IF NOT EXISTS cart_items (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      cart_id INTEGER NOT NULL REFERENCES carts(id) ON DELETE CASCADE,
      product_id INTEGER NOT NULL REFERENCES products(id),
      quantity INTEGER NOT NULL CHECK (quantity > 0),
      UNIQUE(cart_id, product_id)
    );

    CREATE TABLE IF NOT EXISTS orders (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      user_id INTEGER REFERENCES users(id),
      status TEXT NOT NULL,
      total_price INTEGER NOT NULL,
      created_at TEXT NOT NULL,
      updated_at TEXT NOT NULL
    );

    CREATE TABLE IF NOT EXISTS order_items (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      order_id INTEGER NOT NULL REFERENCES orders(id) ON DELETE CASCADE,
      product_id INTEGER NOT NULL REFERENCES products(id),
      product_name TEXT NOT NULL,
      unit_price INTEGER NOT NULL,
      quantity INTEGER NOT NULL CHECK (quantity > 0)
    );

    CREATE TABLE IF NOT EXISTS devices (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      code TEXT NOT NULL UNIQUE,
      type TEXT NOT NULL CHECK (type IN ('cart', 'station')),
      status TEXT NOT NULL CHECK (status IN ('idle', 'busy', 'error', 'offline'))
    );

    CREATE TABLE IF NOT EXISTS missions (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      order_id INTEGER NOT NULL REFERENCES orders(id),
      device_id INTEGER REFERENCES devices(id),
      status TEXT NOT NULL,
      created_at TEXT NOT NULL,
      current_waypoint TEXT,
      current_waypoint_label TEXT
    );

    CREATE TABLE IF NOT EXISTS mission_events (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      mission_id INTEGER NOT NULL REFERENCES missions(id) ON DELETE CASCADE,
      from_status TEXT,
      to_status TEXT NOT NULL,
      note TEXT,
      created_at TEXT NOT NULL
    );

    UPDATE products
    SET image_zoom_url = image_full_url
    WHERE image_full_url LIKE '/uploads/%'
      AND (
        image_zoom_url IS NULL
        OR image_zoom_url = ''
        OR image_zoom_url LIKE '%placehold.co%'
      );
    """
    )
    conn.commit()
    _ensure_column(conn, "missions", "current_waypoint", "TEXT")
    _ensure_column(conn, "missions", "current_waypoint_label", "TEXT")


def _ensure_column(
    conn: LockedConnection, table: str, column: str, col_type: str
) -> None:
    rows = conn.execute(f"PRAGMA table_info({table})").fetchall()
    names = {r["name"] if isinstance(r, sqlite3.Row) else r[1] for r in rows}
    if column not in names:
        conn.execute(f"ALTER TABLE {table} ADD COLUMN {column} {col_type}")
        conn.commit()
