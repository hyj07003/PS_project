import * as bcrypt from "bcryptjs";
import type { DatabaseSync } from "node:sqlite";
import { CATEGORY_SEEDS } from "@smartshop/shared";

function now(): string {
  return new Date().toISOString();
}

function placeholder(label: string, variant: "full" | "zoom"): string {
  const bg = variant === "full" ? "d8d4cc" : "c4bfb5";
  const fg = "2c2c2c";
  const text = encodeURIComponent(label);
  return `https://placehold.co/800x1000/${bg}/${fg}/png?text=${text}`;
}

export function seedIfEmpty(db: DatabaseSync): void {
  const userCount = db.prepare("SELECT COUNT(*) AS c FROM users").get() as {
    c: number;
  };
  if (userCount.c > 0) return;

  const ts = now();
  const adminHash = bcrypt.hashSync("admin1234", 10);
  const customerHash = bcrypt.hashSync("customer1234", 10);

  const insertUser = db.prepare(`
    INSERT INTO users (email, password_hash, name, role, status, created_at, updated_at)
    VALUES (?, ?, ?, ?, 'active', ?, ?)
  `);

  const admin = insertUser.run(
    "admin@smartshop.local",
    adminHash,
    "관리자",
    "admin",
    ts,
    ts,
  );
  insertUser.run(
    "customer@smartshop.local",
    customerHash,
    "고객",
    "customer",
    ts,
    ts,
  );

  const insertCategory = db.prepare(`
    INSERT INTO categories (code, name, sort_order, is_active)
    VALUES (?, ?, ?, 1)
  `);
  for (const cat of CATEGORY_SEEDS) {
    insertCategory.run(cat.code, cat.name, cat.sortOrder);
  }

  const categories = db
    .prepare("SELECT id, code FROM categories")
    .all() as Array<{ id: number; code: string }>;
  const byCode = Object.fromEntries(categories.map((c) => [c.code, c.id]));

  const products = [
    {
      code: "fresh",
      name: "유기농 사과 1kg",
      slug: "organic-apple-1kg",
      description: "아삭한 식감의 국내산 유기농 사과입니다.",
      price: 8900,
      featured: 1,
    },
    {
      code: "fresh",
      name: "신선 시금치 묶음",
      slug: "fresh-spinach",
      description: "당일 수확 시금치, 나물·무침용.",
      price: 3200,
      featured: 0,
    },
    {
      code: "dairy",
      name: "저지방 우유 1L",
      slug: "lowfat-milk-1l",
      description: "고소한 저지방 우유.",
      price: 2800,
      featured: 1,
    },
    {
      code: "dairy",
      name: "그릭 요거트 400g",
      slug: "greek-yogurt-400g",
      description: "진한 그릭 요거트.",
      price: 4500,
      featured: 0,
    },
    {
      code: "beverage",
      name: "스파클링 워터 500ml",
      slug: "sparkling-water-500",
      description: "청량한 탄산수.",
      price: 1500,
      featured: 0,
    },
    {
      code: "beverage",
      name: "콜드브루 커피 1L",
      slug: "coldbrew-1l",
      description: "부드러운 콜드브루.",
      price: 6900,
      featured: 1,
    },
    {
      code: "snack",
      name: "허니버터칩",
      slug: "honey-butter-chips",
      description: "달콤짭짤한 감자칩.",
      price: 2200,
      featured: 0,
    },
    {
      code: "ready",
      name: "즉석 김치찌개",
      slug: "instant-kimchi-jjigae",
      description: "전자레인지 3분 완성.",
      price: 4900,
      featured: 0,
    },
    {
      code: "household",
      name: "키친타월 6롤",
      slug: "kitchen-towel-6",
      description: "두툼한 흡수력.",
      price: 7800,
      featured: 0,
    },
    {
      code: "kitchen",
      name: "친환경 주방세제",
      slug: "eco-dish-soap",
      description: "피부 자극 낮은 주방세제.",
      price: 3900,
      featured: 0,
    },
    {
      code: "health",
      name: "비타민C 100정",
      slug: "vitamin-c-100",
      description: "하루 한 알 비타민C.",
      price: 12900,
      featured: 1,
    },
    {
      code: "snack",
      name: "다크 초콜릿바",
      slug: "dark-chocolate-bar",
      description: "카카오 70% 다크 초콜릿.",
      price: 3500,
      featured: 0,
    },
  ];

  const insertProduct = db.prepare(`
    INSERT INTO products (
      category_id, name, slug, description, price, stock,
      image_full_url, image_zoom_url, is_featured, is_active,
      created_by, created_at, updated_at
    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, 1, ?, ?, ?)
  `);

  for (const p of products) {
    insertProduct.run(
      byCode[p.code],
      p.name,
      p.slug,
      p.description,
      p.price,
      50,
      placeholder(p.name, "full"),
      placeholder(`${p.name}+`, "zoom"),
      p.featured,
      admin.lastInsertRowid,
      ts,
      ts,
    );
  }

  const insertDevice = db.prepare(
    `INSERT INTO devices (code, type, status) VALUES (?, ?, 'idle')`,
  );
  insertDevice.run("cart-1", "cart");
  insertDevice.run("cart-2", "cart");
  insertDevice.run("station-1", "station");
  insertDevice.run("station-2", "station");
}
