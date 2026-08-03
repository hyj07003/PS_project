"use client";

import Link from "next/link";
import { usePathname } from "next/navigation";
import { useEffect, useState } from "react";
import type { Category } from "@smartshop/shared";
import { api } from "@/lib/api";
import { useAuth } from "@/lib/auth-context";

export function SiteHeader() {
  const { user, cartCount, logout } = useAuth();
  const pathname = usePathname();
  const [categories, setCategories] = useState<Category[]>([]);
  const [open, setOpen] = useState(false);

  useEffect(() => {
    api<Category[]>("/categories", { auth: false })
      .then(setCategories)
      .catch(() => setCategories([]));
  }, []);

  useEffect(() => {
    setOpen(false);
  }, [pathname]);

  return (
    <header className="site-header">
      <div className="container nav">
        <Link href="/" className="brand">
          SmartShop
        </Link>
        <nav className="nav-cats" aria-label="카테고리">
          {categories.map((c) => (
            <Link
              key={c.id}
              href={`/?category=${c.code}`}
              className={pathname === "/" ? undefined : undefined}
            >
              {c.name}
            </Link>
          ))}
        </nav>
        <div className="nav-actions">
          <button
            type="button"
            className="menu-toggle"
            onClick={() => setOpen((v) => !v)}
            aria-label="메뉴"
          >
            메뉴
          </button>
          <Link href="/cart" className="cart-badge" aria-label="장바구니">
            Bag
            {cartCount > 0 ? <span>{cartCount}</span> : null}
          </Link>
          {user?.role === "admin" ? (
            <Link href="/admin">매장 관리</Link>
          ) : null}
          {user ? (
            <>
              <span className="muted">{user.name}</span>
              <button type="button" onClick={logout}>
                로그아웃
              </button>
            </>
          ) : (
            <>
              <Link href="/login">로그인</Link>
              <Link href="/register">회원가입</Link>
            </>
          )}
        </div>
      </div>
      <div className={`container mobile-drawer ${open ? "open" : ""}`}>
        {categories.map((c) => (
          <Link key={c.id} href={`/?category=${c.code}`}>
            {c.name}
          </Link>
        ))}
      </div>
    </header>
  );
}
