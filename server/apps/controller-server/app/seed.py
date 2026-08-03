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

PRODUCTS = [
    {
        "code": "fresh",
        "name": "유기농 사과 1kg",
        "slug": "organic-apple-1kg",
        "description": "아삭한 식감의 국내산 유기농 사과입니다.",
        "price": 8900,
        "featured": 1,
    },
    {
        "code": "fresh",
        "name": "신선 시금치 묶음",
        "slug": "fresh-spinach",
        "description": "당일 수확 시금치, 나물·무침용.",
        "price": 3200,
        "featured": 0,
    },
    {
        "code": "dairy",
        "name": "저지방 우유 1L",
        "slug": "lowfat-milk-1l",
        "description": "고소한 저지방 우유.",
        "price": 2800,
        "featured": 1,
    },
    {
        "code": "dairy",
        "name": "그릭 요거트 400g",
        "slug": "greek-yogurt-400g",
        "description": "진한 그릭 요거트.",
        "price": 4500,
        "featured": 0,
    },
    {
        "code": "beverage",
        "name": "스파클링 워터 500ml",
        "slug": "sparkling-water-500",
        "description": "청량한 탄산수.",
        "price": 1500,
        "featured": 0,
    },
    {
        "code": "beverage",
        "name": "콜드브루 커피 1L",
        "slug": "coldbrew-1l",
        "description": "부드러운 콜드브루.",
        "price": 6900,
        "featured": 1,
    },
    {
        "code": "snack",
        "name": "허니버터칩",
        "slug": "honey-butter-chips",
        "description": "달콤짭짤한 감자칩.",
        "price": 2200,
        "featured": 0,
    },
    {
        "code": "ready",
        "name": "즉석 김치찌개",
        "slug": "instant-kimchi-jjigae",
        "description": "전자레인지 3분 완성.",
        "price": 4900,
        "featured": 0,
    },
    {
        "code": "household",
        "name": "키친타월 6롤",
        "slug": "kitchen-towel-6",
        "description": "두툼한 흡수력.",
        "price": 7800,
        "featured": 0,
    },
    {
        "code": "kitchen",
        "name": "친환경 주방세제",
        "slug": "eco-dish-soap",
        "description": "피부 자극 낮은 주방세제.",
        "price": 3900,
        "featured": 0,
    },
    {
        "code": "health",
        "name": "비타민C 100정",
        "slug": "vitamin-c-100",
        "description": "하루 한 알 비타민C.",
        "price": 12900,
        "featured": 1,
    },
    {
        "code": "snack",
        "name": "다크 초콜릿바",
        "slug": "dark-chocolate-bar",
        "description": "카카오 70% 다크 초콜릿.",
        "price": 3500,
        "featured": 0,
    },
]


def seed_if_empty(conn) -> None:
    row = conn.execute("SELECT COUNT(*) AS c FROM users").fetchone()
    if row["c"] > 0:
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

    categories = conn.execute("SELECT id, code FROM categories").fetchall()
    by_code = {c["code"]: c["id"] for c in categories}

    for p in PRODUCTS:
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

    for code, dtype in (
        ("cart-1", "cart"),
        ("cart-2", "cart"),
        ("station-1", "station"),
        ("station-2", "station"),
    ):
        conn.execute(
            "INSERT INTO devices (code, type, status) VALUES (?, ?, 'idle')",
            (code, dtype),
        )

    conn.commit()
