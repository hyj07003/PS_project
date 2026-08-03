from __future__ import annotations

import sqlite3

import bcrypt

from ..db import map_user, now_iso
from ..errors import ApiError


class UsersService:
    def __init__(self, conn: sqlite3.Connection):
        self.conn = conn

    def find_by_email(self, email: str) -> sqlite3.Row | None:
        return self.conn.execute(
            "SELECT * FROM users WHERE email = ?",
            (email.lower(),),
        ).fetchone()

    def find_by_id(self, user_id: int) -> dict | None:
        row = self.conn.execute(
            "SELECT * FROM users WHERE id = ?",
            (user_id,),
        ).fetchone()
        return map_user(dict(row)) if row else None

    def register(self, email: str, password: str, name: str) -> dict:
        if not email or not password or not name:
            raise ApiError(400, "email, password, name are required")
        if len(password) < 6:
            raise ApiError(400, "password must be at least 6 characters")
        if self.find_by_email(email):
            raise ApiError(400, "email already registered")

        ts = now_iso()
        pw_hash = bcrypt.hashpw(password.encode(), bcrypt.gensalt(rounds=10)).decode()
        cur = self.conn.execute(
            """
            INSERT INTO users (email, password_hash, name, role, status, created_at, updated_at)
            VALUES (?, ?, ?, 'customer', 'active', ?, ?)
            """,
            (email.lower(), pw_hash, name, ts, ts),
        )
        user_id = cur.lastrowid
        self.conn.execute(
            "INSERT INTO carts (user_id, updated_at) VALUES (?, ?)",
            (user_id, ts),
        )
        self.conn.commit()
        return self.find_by_id(user_id)  # type: ignore[return-value]

    def verify_login(self, email: str, password: str) -> dict:
        row = self.find_by_email(email)
        if not row or row["status"] != "active":
            raise ApiError(401, "invalid credentials")
        if not bcrypt.checkpw(password.encode(), row["password_hash"].encode()):
            raise ApiError(401, "invalid credentials")
        return map_user(dict(row))
