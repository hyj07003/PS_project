from __future__ import annotations

import sqlite3

import bcrypt

from .db import now_iso, placeholder

CATEGORY_SEEDS = [
    {"code": "fresh", "name": "신선식품", "sortOrder": 1},
    {"code": "dairy", "name": "유제품", "sortOrder": 2},
    {"code": "beverage", "name": "음료", "sortOrder": 3},
    {"code": "snack", "name": "스낵/과자", "sortOrder": 4},
    {"code": "ready", "name": "즉석식품", "sortOrder": 5},
    {"code": "household", "name": "생활용품", "sortOrder": 6},
    {"code": "kitchen", "name": "주방/세제", "sortOrder": 7},
    {"code": "health", "name": "헬스/뷰티", "sortOrder": 8},
]

# Demo catalog for map waypoints W1–W6
PRODUCTS = [
    {
        "code": "ready",
        "name": "케이크",
        "slug": "cake",
        "description": "매장 데모 — 케이크 매대(W1).",
        "price": 12000,
        "featured": 1,
    },
    {
        "code": "ready",
        "name": "롤케이크",
        "slug": "roll-cake",
        "description": "매장 데모 — 롤케이크 매대(W2).",
        "price": 8900,
        "featured": 1,
    },
    {
        "code": "ready",
        "name": "우유",
        "slug": "milk",
        "description": "매장 데모 — 우유 매대(W3).",
        "price": 2800,
        "featured": 1,
    },
    {
        "code": "snack",
        "name": "비스킷",
        "slug": "biscuit",
        "description": "매장 데모 — 비스킷 매대(W4).",
        "price": 3000,
        "featured": 1,
    },
    {
        "code": "dairy",
        "name": "아이스크림",
        "slug": "ice-cream",
        "description": "매장 데모 — 아이스크림 매대(W5).",
        "price": 3500,
        "featured": 1,
    },
    {
        "code": "beverage",
        "name": "샌드위치",
        "slug": "sandwich",
        "description": "매장 데모 — 샌드위치 매대(W6).",
        "price": 5500,
        "featured": 1,
    },
]

DEMO_SLUGS = {p["slug"] for p in PRODUCTS}


def seed_if_empty(conn) -> None:
    row = conn.execute("SELECT COUNT(*) AS c FROM users").fetchone()
    if row["c"] > 0:
        ensure_demo_catalog(conn)
        return

    ts = now_iso()
    admin_hash = bcrypt.hashpw(b"admin1234", bcrypt.gensalt(rounds=10)).decode()
    customer_hash = bcrypt.hashpw(
        b"customer1234", bcrypt.gensalt(rounds=10)
    ).decode()

    cur = conn.execute(
        """
        INSERT INTO users (email, password_hash, name, role, status, created_at, updated_at)
        VALUES (?, ?, ?, ?, 'active', ?, ?)
        """,
        ("admin@smartshop.local", admin_hash, "관리자", "admin", ts, ts),
    )
    admin_id = cur.lastrowid
    conn.execute(
        """
        INSERT INTO users (email, password_hash, name, role, status, created_at, updated_at)
        VALUES (?, ?, ?, ?, 'active', ?, ?)
        """,
        (
            "customer@smartshop.local",
            customer_hash,
            "고객",
            "customer",
            ts,
            ts,
        ),
    )

    for cat in CATEGORY_SEEDS:
        conn.execute(
            "INSERT INTO categories (code, name, sort_order, is_active) VALUES (?, ?, ?, 1)",
            (cat["code"], cat["name"], cat["sortOrder"]),
        )

    _insert_demo_products(conn, admin_id, ts)
    _ensure_devices(conn)
    conn.commit()


def ensure_demo_catalog(conn) -> None:
    """Upsert demo products and deactivate non-demo SKUs on existing DBs."""
    ts = now_iso()
    admin = conn.execute(
        "SELECT id FROM users WHERE role = 'admin' ORDER BY id ASC LIMIT 1"
    ).fetchone()
    admin_id = admin["id"] if admin else None

    for cat in CATEGORY_SEEDS:
        conn.execute(
            """
            INSERT INTO categories (code, name, sort_order, is_active)
            VALUES (?, ?, ?, 1)
            ON CONFLICT(code) DO UPDATE SET
              name = excluded.name,
              sort_order = excluded.sort_order,
              is_active = 1
            """,
            (cat["code"], cat["name"], cat["sortOrder"]),
        )

    _insert_demo_products(conn, admin_id, ts)
    _ensure_devices(conn)
    conn.commit()


def _is_custom_image(url: str | None) -> bool:
    text = (url or "").strip()
    if not text:
        return False
    if "placehold.co" in text.lower():
        return False
    return True


def _insert_demo_products(conn, admin_id: int | None, ts: str) -> None:
    categories = conn.execute("SELECT id, code FROM categories").fetchall()
    by_code = {c["code"]: c["id"] for c in categories}

    for p in PRODUCTS:
        existing = conn.execute(
            "SELECT id, image_full_url, image_zoom_url FROM products WHERE slug = ?",
            (p["slug"],),
        ).fetchone()
        default_full = placeholder(p["name"], "full")
        default_zoom = placeholder(f"{p['name']}+", "zoom")
        if existing:
            full = (
                existing["image_full_url"]
                if _is_custom_image(existing["image_full_url"])
                else default_full
            )
            zoom = (
                existing["image_zoom_url"]
                if _is_custom_image(existing["image_zoom_url"])
                else (
                    full
                    if _is_custom_image(full)
                    else default_zoom
                )
            )
            conn.execute(
                """
                UPDATE products SET
                  category_id = ?, name = ?, description = ?, price = ?,
                  stock = CASE WHEN stock < 50 THEN 50 ELSE stock END,
                  image_full_url = ?, image_zoom_url = ?,
                  is_featured = ?, is_active = 1, updated_at = ?
                WHERE slug = ?
                """,
                (
                    by_code[p["code"]],
                    p["name"],
                    p["description"],
                    p["price"],
                    full,
                    zoom,
                    p["featured"],
                    ts,
                    p["slug"],
                ),
            )
        else:
            conn.execute(
                """
                INSERT INTO products (
                  category_id, name, slug, description, price, stock,
                  image_full_url, image_zoom_url, is_featured, is_active,
                  created_by, created_at, updated_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, 1, ?, ?, ?)
                """,
                (
                    by_code[p["code"]],
                    p["name"],
                    p["slug"],
                    p["description"],
                    p["price"],
                    50,
                    placeholder(p["name"], "full"),
                    placeholder(f"{p['name']}+", "zoom"),
                    p["featured"],
                    admin_id,
                    ts,
                    ts,
                ),
            )


def _ensure_devices(conn) -> None:
    for code, dtype in (
        ("cart-1", "cart"),
        ("cart-2", "cart"),
        ("station-1", "station"),
        ("station-2", "station"),
    ):
        conn.execute(
            """
            INSERT INTO devices (code, type, status)
            VALUES (?, ?, 'idle')
            ON CONFLICT(code) DO NOTHING
            """,
            (code, dtype),
        )
