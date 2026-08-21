"use client";

import { useEffect, useState } from "react";
import { useParams } from "next/navigation";
import type { Product } from "@smartshop/shared";
import { formatPriceKrw } from "@smartshop/shared";
import { api, resolveMediaUrl } from "@/lib/api";
import { useAuth } from "@/lib/auth-context";

export default function ProductDetailPage() {
  const params = useParams<{ id: string }>();
  const { addToCart } = useAuth();
  const [product, setProduct] = useState<Product | null>(null);
  const [qty, setQty] = useState(1);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    api<Product>(`/products/${params.id}`, { auth: false })
      .then(setProduct)
      .catch((e: Error) => setError(e.message));
  }, [params.id]);

  if (error) return <div className="container page error">{error}</div>;
  if (!product) return <div className="container page muted">불러오는 중…</div>;

  return (
    <div className="container page">
      <div
        style={{
          display: "grid",
          gridTemplateColumns: "repeat(auto-fit, minmax(280px, 1fr))",
          gap: "1.5rem",
        }}
      >
        <div className="hero-visual" style={{ minHeight: 420 }}>
          <div className="pan-frame">
            <img
              src={resolveMediaUrl(
                product.imageZoomUrl &&
                  product.imageZoomUrl !== product.imageFullUrl &&
                  !/placehold\.co/i.test(product.imageZoomUrl)
                  ? product.imageZoomUrl
                  : product.imageFullUrl || product.imageZoomUrl,
              )}
              alt={`${product.name} 확대`}
              className={
                !product.imageZoomUrl ||
                product.imageZoomUrl === product.imageFullUrl ||
                /placehold\.co/i.test(product.imageZoomUrl || "")
                  ? "media-zoom"
                  : undefined
              }
            />
          </div>
          <div className="pan-frame">
            <img
              src={resolveMediaUrl(
                product.imageFullUrl || product.imageZoomUrl,
              )}
              alt={`${product.name} 전체`}
            />
          </div>
        </div>
        <div className="stack">
          <p className="hero-kicker">{product.categoryName}</p>
          <h1 className="hero-title" style={{ fontSize: "clamp(2rem,4vw,3rem)" }}>
            {product.name}
          </h1>
          <p className="hero-price">{formatPriceKrw(product.price)}</p>
          <p className="muted">{product.description}</p>
          <p className="muted">재고 · {product.stock}</p>
          <label>
            수량
            <input
              type="number"
              min={1}
              max={Math.max(1, Math.min(3, product.stock))}
              value={qty}
              onChange={(e) => {
                const max = Math.max(0, Math.min(3, product.stock));
                const next = Number(e.target.value) || 1;
                setQty(Math.max(1, Math.min(max || 1, next)));
              }}
              style={{ marginLeft: "0.75rem", width: 80, padding: "0.4rem" }}
            />
          </label>
          <button
            type="button"
            className="btn"
            disabled={product.stock < 1}
            onClick={() => void addToCart(product.id, qty)}
          >
            {product.stock < 1 ? "품절" : "장바구니 담기"}
          </button>
        </div>
      </div>
    </div>
  );
}
