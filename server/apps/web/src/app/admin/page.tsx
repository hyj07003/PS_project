"use client";

import { FormEvent, useEffect, useState } from "react";
import { useRouter } from "next/navigation";
import type { Category, CreateProductInput, Product } from "@smartshop/shared";
import { formatPriceKrw } from "@smartshop/shared";
import { api, getApiUrl, getToken } from "@/lib/api";
import { useAuth } from "@/lib/auth-context";

const emptyForm: CreateProductInput = {
  categoryId: 1,
  name: "",
  description: "",
  price: 0,
  stock: 10,
  imageFullUrl: "",
  imageZoomUrl: "",
  isFeatured: false,
  isActive: true,
};

export default function AdminPage() {
  const { user, loading } = useAuth();
  const router = useRouter();
  const [products, setProducts] = useState<Product[]>([]);
  const [categories, setCategories] = useState<Category[]>([]);
  const [form, setForm] = useState<CreateProductInput>(emptyForm);
  const [editingId, setEditingId] = useState<number | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [message, setMessage] = useState<string | null>(null);

  const load = async () => {
    const [p, c] = await Promise.all([
      api<Product[]>("/admin/products"),
      api<Category[]>("/categories", { auth: false }),
    ]);
    setProducts(p);
    setCategories(c);
    if (c[0] && !editingId) {
      setForm((f) => ({ ...f, categoryId: c[0].id }));
    }
  };

  useEffect(() => {
    if (loading) return;
    if (!user) {
      router.replace("/login");
      return;
    }
    if (user.role !== "admin") {
      router.replace("/");
      return;
    }
    void load().catch((e: Error) => setError(e.message));
  }, [user, loading, router]);

  const upload = async (file: File, field: "imageFullUrl" | "imageZoomUrl") => {
    const body = new FormData();
    body.append("file", file);
    const res = await fetch(`${getApiUrl()}/admin/upload`, {
      method: "POST",
      headers: {
        Authorization: `Bearer ${getToken()}`,
      },
      body,
    });
    const data = (await res.json()) as { url?: string; message?: string };
    if (!res.ok) throw new Error(data.message || "upload failed");
    const url = data.url || "";
    setForm((f) => {
      if (field === "imageFullUrl") {
        // 전체 이미지 등록 시 확대 이미지도 동일 소스 사용
        return { ...f, imageFullUrl: url, imageZoomUrl: url };
      }
      return {
        ...f,
        imageZoomUrl: url,
        imageFullUrl: f.imageFullUrl || url,
      };
    });
  };

  const onSubmit = async (e: FormEvent) => {
    e.preventDefault();
    setError(null);
    setMessage(null);
    const full = (form.imageFullUrl || "").trim();
    const zoom = (form.imageZoomUrl || "").trim();
    const payload: CreateProductInput = {
      ...form,
      imageFullUrl: full || zoom || "",
      // 확대 미지정·플레이스홀더 잔존 시 전체 이미지 사용
      imageZoomUrl:
        !zoom || /placehold\.co/i.test(zoom) || (full.startsWith("/uploads/") && zoom.startsWith("http"))
          ? full || zoom
          : zoom,
    };
    if (!payload.imageFullUrl && payload.imageZoomUrl) {
      payload.imageFullUrl = payload.imageZoomUrl;
    }
    try {
      if (editingId) {
        await api(`/admin/products/${editingId}`, {
          method: "PUT",
          body: JSON.stringify(payload),
        });
        setMessage("상품이 수정되었습니다.");
      } else {
        await api("/admin/products", {
          method: "POST",
          body: JSON.stringify(payload),
        });
        setMessage("상품이 등록되었습니다.");
      }
      setEditingId(null);
      setForm({
        ...emptyForm,
        categoryId: categories[0]?.id || 1,
      });
      await load();
    } catch (err) {
      setError(err instanceof Error ? err.message : "저장 실패");
    }
  };

  const edit = (p: Product) => {
    setEditingId(p.id);
    setForm({
      categoryId: p.categoryId,
      name: p.name,
      slug: p.slug,
      description: p.description || "",
      price: p.price,
      stock: p.stock,
      imageFullUrl: p.imageFullUrl || "",
      imageZoomUrl: p.imageZoomUrl || "",
      isFeatured: p.isFeatured,
      isActive: p.isActive,
    });
  };

  const remove = async (id: number) => {
    if (!confirm("삭제할까요?")) return;
    await api(`/admin/products/${id}`, { method: "DELETE" });
    await load();
  };

  if (loading || !user || user.role !== "admin") {
    return <div className="container page muted">확인 중…</div>;
  }

  return (
    <div className="container page">
      <h1 className="hero-title" style={{ fontSize: "2.4rem" }}>
        상품 관리
      </h1>
      <p className="muted">관리자만 접근 가능한 상품 등록/수정 화면입니다.</p>
      {error ? <p className="error">{error}</p> : null}
      {message ? <p>{message}</p> : null}

      <form className="form" style={{ maxWidth: 640 }} onSubmit={onSubmit}>
        <label>
          상품명
          <input
            value={form.name}
            onChange={(e) => setForm({ ...form, name: e.target.value })}
            required
          />
        </label>
        <label>
          카테고리
          <select
            value={form.categoryId}
            onChange={(e) =>
              setForm({ ...form, categoryId: Number(e.target.value) })
            }
          >
            {categories.map((c) => (
              <option key={c.id} value={c.id}>
                {c.name}
              </option>
            ))}
          </select>
        </label>
        <label>
          가격(원)
          <input
            type="number"
            value={form.price}
            onChange={(e) =>
              setForm({ ...form, price: Number(e.target.value) || 0 })
            }
            required
          />
        </label>
        <label>
          재고
          <input
            type="number"
            value={form.stock ?? 0}
            onChange={(e) =>
              setForm({ ...form, stock: Number(e.target.value) || 0 })
            }
          />
        </label>
        <label>
          설명
          <textarea
            rows={3}
            value={form.description || ""}
            onChange={(e) => setForm({ ...form, description: e.target.value })}
          />
        </label>
        <label>
          전체 이미지 URL
          <input
            value={form.imageFullUrl || ""}
            onChange={(e) => {
              const imageFullUrl = e.target.value;
              setForm((f) => ({
                ...f,
                imageFullUrl,
                // 전체 이미지 URL 입력 시 확대도 같이 맞춤
                imageZoomUrl: imageFullUrl,
              }));
            }}
          />
        </label>
        <label>
          전체 이미지 업로드
          <input
            type="file"
            accept="image/*"
            onChange={(e) => {
              const file = e.target.files?.[0];
              if (file) void upload(file, "imageFullUrl").catch((err: Error) =>
                setError(err.message),
              );
            }}
          />
          <span className="muted" style={{ fontSize: "0.85rem" }}>
            이미지가 하나면 확대 이미지로도 자동 사용됩니다.
          </span>
        </label>
        <label>
          확대 이미지 URL (비우면 전체 이미지 사용)
          <input
            value={form.imageZoomUrl || ""}
            onChange={(e) => setForm({ ...form, imageZoomUrl: e.target.value })}
            placeholder="비워두면 전체 이미지를 확대해 사용"
          />
        </label>
        <label>
          확대 이미지 업로드
          <input
            type="file"
            accept="image/*"
            onChange={(e) => {
              const file = e.target.files?.[0];
              if (file) void upload(file, "imageZoomUrl").catch((err: Error) =>
                setError(err.message),
              );
            }}
          />
        </label>
        <label style={{ display: "flex", gap: "0.5rem", alignItems: "center" }}>
          <input
            type="checkbox"
            checked={!!form.isFeatured}
            onChange={(e) => setForm({ ...form, isFeatured: e.target.checked })}
          />
          히어로 슬라이드 노출
        </label>
        <label style={{ display: "flex", gap: "0.5rem", alignItems: "center" }}>
          <input
            type="checkbox"
            checked={form.isActive !== false}
            onChange={(e) => setForm({ ...form, isActive: e.target.checked })}
          />
          판매 중
        </label>
        <div style={{ display: "flex", gap: "0.75rem" }}>
          <button className="btn" type="submit">
            {editingId ? "수정 저장" : "상품 등록"}
          </button>
          {editingId ? (
            <button
              type="button"
              className="btn secondary"
              onClick={() => {
                setEditingId(null);
                setForm({
                  ...emptyForm,
                  categoryId: categories[0]?.id || 1,
                });
              }}
            >
              취소
            </button>
          ) : null}
        </div>
      </form>

      <h2 style={{ marginTop: "2.5rem" }}>등록 상품</h2>
      <table className="table">
        <thead>
          <tr>
            <th>ID</th>
            <th>이름</th>
            <th>가격</th>
            <th>히어로</th>
            <th>활성</th>
            <th />
          </tr>
        </thead>
        <tbody>
          {products.map((p) => (
            <tr key={p.id}>
              <td>{p.id}</td>
              <td>{p.name}</td>
              <td>{formatPriceKrw(p.price)}</td>
              <td>{p.isFeatured ? "Y" : "-"}</td>
              <td>{p.isActive ? "Y" : "N"}</td>
              <td style={{ display: "flex", gap: "0.5rem" }}>
                <button type="button" className="btn ghost" onClick={() => edit(p)}>
                  수정
                </button>
                <button
                  type="button"
                  className="btn secondary"
                  onClick={() => void remove(p.id)}
                >
                  삭제
                </button>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}
