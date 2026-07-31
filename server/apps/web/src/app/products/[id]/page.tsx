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
          <label>
            수량
            <input
              type="number"
              min={1}
              value={qty}
              onChange={(e) => setQty(Number(e.target.value) || 1)}
              style={{ marginLeft: "0.75rem", width: 80, padding: "0.4rem" }}
            />
          </label>
          <button
            type="button"
            className="btn"
            onClick={() => void addToCart(product.id, qty)}
          >
            장바구니 담기
          </button>
        </div>
      </div>
    </div>
  );
}
