"use client";

import Link from "next/link";
import { useMemo, useState } from "react";
import type { Product } from "@smartshop/shared";
import { formatPriceKrw } from "@smartshop/shared";
import { resolveMediaUrl } from "@/lib/api";
import { useAuth } from "@/lib/auth-context";

export function ProductGrid({ products }: { products: Product[] }) {
  const { addToCart } = useAuth();
  const [activeId, setActiveId] = useState<number | null>(null);

  const ordered = useMemo(() => {
    const featured = products.find((p) => p.isFeatured) || products[0];
    const rest = products.filter((p) => p.id !== featured?.id);
    return featured ? [featured, ...rest] : rest;
  }, [products]);

  if (!ordered.length) {
    return <p className="muted">검색 결과가 없습니다.</p>;
  }

  return (
    <div className="product-grid">
      {ordered.map((p, idx) => (
        <article
          key={p.id}
          className={`product-card ${idx === 0 ? "featured" : ""} ${
            activeId === p.id ? "active" : ""
          }`}
          onClick={() => setActiveId((id) => (id === p.id ? null : p.id))}
        >
          <img
            src={resolveMediaUrl(p.imageFullUrl || p.imageZoomUrl)}
            alt={p.name}
          />
          <div className="product-meta">
            <h3>{p.name}</h3>
            <p>{formatPriceKrw(p.price)}</p>
          </div>
          <div className="card-actions">
            <Link
              href={`/products/${p.id}`}
              className="btn ghost"
              onClick={(e) => e.stopPropagation()}
            >
              상세
            </Link>
            <button
              type="button"
              className="btn"
              onClick={(e) => {
                e.stopPropagation();
                void addToCart(p.id);
              }}
            >
              장바구니
            </button>
          </div>
        </article>
      ))}
    </div>
  );
}
