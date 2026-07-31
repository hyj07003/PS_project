"use client";

import { AnimatePresence, motion } from "framer-motion";
import { useEffect, useMemo, useState } from "react";
import type { Product } from "@smartshop/shared";
import { formatPriceKrw } from "@smartshop/shared";
import { resolveMediaUrl } from "@/lib/api";
import { useAuth } from "@/lib/auth-context";

export function HeroSlider({ products }: { products: Product[] }) {
  const slides = useMemo(() => {
    const featured = products.filter((p) => p.isFeatured);
    return (featured.length ? featured : products).slice(0, 6);
  }, [products]);
  const [index, setIndex] = useState(0);
  const { addToCart } = useAuth();

  useEffect(() => {
    if (slides.length <= 1) return;
    const id = window.setInterval(() => {
      setIndex((i) => (i + 1) % slides.length);
    }, 6500);
    return () => window.clearInterval(id);
  }, [slides.length]);

  if (!slides.length) {
    return (
      <section className="hero container">
        <div className="hero-copy">
          <p className="hero-kicker">SmartShop</p>
          <h1 className="hero-title">등록된 상품이 없습니다</h1>
        </div>
      </section>
    );
  }

  const current = slides[index];
  const fullUrl = current.imageFullUrl || current.imageZoomUrl;
  const hasDedicatedZoom =
    !!current.imageZoomUrl &&
    !!current.imageFullUrl &&
    current.imageZoomUrl !== current.imageFullUrl &&
    !/placehold\.co/i.test(current.imageZoomUrl);
  const zoomUrl = hasDedicatedZoom ? current.imageZoomUrl : fullUrl;

  return (
    <section className="hero container">
      <div className="hero-copy">
        <p className="hero-kicker">{current.categoryName || "COLLECTION"}</p>
        <AnimatePresence mode="wait">
          <motion.div
            key={current.id}
            initial={{ opacity: 0, y: 12 }}
            animate={{ opacity: 1, y: 0 }}
            exit={{ opacity: 0, y: -8 }}
            transition={{ duration: 0.45 }}
          >
            <h1 className="hero-title">{current.name}</h1>
            <div className="hero-meta">
              <div>카테고리 · {current.categoryName}</div>
              <div>재고 · {current.stock}</div>
            </div>
            <p className="hero-price">{formatPriceKrw(current.price)}</p>
            <button
              type="button"
              className="btn"
              onClick={() => void addToCart(current.id)}
            >
              ADD TO CART →
            </button>
          </motion.div>
        </AnimatePresence>
      </div>

      <div className="hero-visual">
        <AnimatePresence mode="wait">
          <motion.div
            key={`zoom-${current.id}`}
            className="pan-frame"
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            transition={{ duration: 0.5 }}
          >
            <img
              src={resolveMediaUrl(zoomUrl)}
              alt={`${current.name} 확대`}
              className={hasDedicatedZoom ? undefined : "media-zoom"}
            />
          </motion.div>
        </AnimatePresence>
        <AnimatePresence mode="wait">
          <motion.div
            key={`full-${current.id}`}
            className="pan-frame"
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            transition={{ duration: 0.5, delay: 0.05 }}
          >
            <img
              src={resolveMediaUrl(fullUrl)}
              alt={`${current.name} 전체`}
            />
          </motion.div>
        </AnimatePresence>
      </div>

      <div className="hero-thumbs" aria-label="히어로 상품 선택">
        {slides.map((p, i) => (
          <button
            key={p.id}
            type="button"
            className={i === index ? "active" : ""}
            onClick={() => setIndex(i)}
          >
            <img
              src={resolveMediaUrl(p.imageFullUrl || p.imageZoomUrl)}
              alt={p.name}
            />
          </button>
        ))}
      </div>
    </section>
  );
}
