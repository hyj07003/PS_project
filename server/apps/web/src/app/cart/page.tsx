"use client";

import Link from "next/link";
import { useEffect, useState } from "react";
import type { Cart, Order, Product } from "@smartshop/shared";
import { formatPriceKrw } from "@smartshop/shared";
import { api } from "@/lib/api";
import {
  getGuestCart,
  updateGuestItem,
  useAuth,
} from "@/lib/auth-context";

type Line = {
  productId: number;
  quantity: number;
  product?: Product;
};

export default function CartPage() {
  const { user, refreshCartCount } = useAuth();
  const [lines, setLines] = useState<Line[]>([]);
  const [error, setError] = useState<string | null>(null);
  const [message, setMessage] = useState<string | null>(null);
  const [pending, setPending] = useState(false);

  const load = async () => {
    setError(null);
    try {
      if (user) {
        const cart = await api<Cart>("/cart");
        setLines(
          cart.items.map((i) => ({
            productId: i.productId,
            quantity: i.quantity,
            product: i.product,
          })),
        );
      } else {
        const guest = getGuestCart();
        const products = await Promise.all(
          guest.map(async (g) => {
            try {
              const p = await api<Product>(`/products/${g.productId}`, {
                auth: false,
              });
              return { ...g, product: p };
            } catch {
              return { ...g, product: undefined };
            }
          }),
        );
        setLines(products);
      }
    } catch (e) {
      setError(e instanceof Error ? e.message : "장바구니 로드 실패");
    }
  };

  useEffect(() => {
    void load();
  }, [user]);

  const total = lines.reduce(
    (sum, line) => sum + (line.product?.price ?? 0) * line.quantity,
    0,
  );

  const changeQty = async (productId: number, quantity: number) => {
    setError(null);
    try {
      if (user) {
        if (quantity <= 0) {
          await api(`/cart/items/${productId}`, { method: "DELETE" });
        } else {
          await api(`/cart/items/${productId}`, {
            method: "PATCH",
            body: JSON.stringify({ quantity }),
          });
        }
        await refreshCartCount();
      } else {
        updateGuestItem(productId, quantity);
        await refreshCartCount();
      }
      await load();
    } catch (e) {
      setError(e instanceof Error ? e.message : "수량 변경 실패");
    }
  };

  const removeItem = async (productId: number) => {
    setError(null);
    try {
      if (user) {
        await api(`/cart/items/${productId}`, { method: "DELETE" });
        await refreshCartCount();
      } else {
        updateGuestItem(productId, 0);
        await refreshCartCount();
      }
      await load();
    } catch (e) {
      setError(e instanceof Error ? e.message : "상품 삭제 실패");
    }
  };

  const checkout = async () => {
    if (!user) {
      setMessage("주문하려면 로그인이 필요합니다.");
      return;
    }
    setPending(true);
    setMessage(null);
    try {
      const order = await api<Order>("/orders", { method: "POST" });
      setMessage(`주문이 생성되었습니다. #${order.id} · ${order.status}`);
      await refreshCartCount();
      await load();
    } catch (e) {
      setError(e instanceof Error ? e.message : "주문 실패");
    } finally {
      setPending(false);
    }
  };

  return (
    <div className="container page">
      <h1 className="hero-title" style={{ fontSize: "2.4rem" }}>
        장바구니
      </h1>
      {!user ? (
        <p className="muted">
          비로그인 상태입니다. 장바구니는 브라우저에 저장됩니다.{" "}
          <Link href="/login">로그인</Link> 시 계정으로 병합됩니다.
        </p>
      ) : null}
      {error ? <p className="error">{error}</p> : null}
      {message ? <p>{message}</p> : null}
      {!lines.length ? (
        <p className="muted">장바구니가 비어 있습니다.</p>
      ) : (
        <>
          <table className="table">
            <thead>
              <tr>
                <th>상품</th>
                <th>가격</th>
                <th>수량</th>
                <th>합계</th>
                <th></th>
              </tr>
            </thead>
            <tbody>
              {lines.map((line) => (
                <tr key={line.productId}>
                  <td>{line.product?.name || `#${line.productId}`}</td>
                  <td>
                    {line.product ? formatPriceKrw(line.product.price) : "-"}
                  </td>
                  <td>
                    <input
                      type="number"
                      min={1}
                      max={Math.max(
                        1,
                        Math.min(3, line.product?.stock ?? 3),
                      )}
                      value={line.quantity}
                      onChange={(e) => {
                        const max = Math.max(
                          1,
                          Math.min(3, line.product?.stock ?? 3),
                        );
                        void changeQty(
                          line.productId,
                          Math.max(1, Math.min(max, Number(e.target.value) || 1)),
                        );
                      }}
                      style={{ width: 72, padding: "0.35rem" }}
                    />
                  </td>
                  <td>
                    {line.product
                      ? formatPriceKrw(line.product.price * line.quantity)
                      : "-"}
                  </td>
                  <td>
                    <button
                      type="button"
                      className="btn secondary"
                      onClick={() => void removeItem(line.productId)}
                      aria-label={`${line.product?.name || line.productId} 삭제`}
                    >
                      삭제
                    </button>
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
          <p className="hero-price" style={{ fontSize: "1.8rem" }}>
            총 {formatPriceKrw(total)}
          </p>
          <button
            type="button"
            className="btn"
            disabled={pending}
            onClick={() => void checkout()}
          >
            {pending ? "주문 중…" : "주문하기"}
          </button>
        </>
      )}
    </div>
  );
}
