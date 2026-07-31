import { Suspense } from "react";
import type { Category, Product } from "@smartshop/shared";
import { HeroSlider } from "@/components/HeroSlider";
import { ProductGrid } from "@/components/ProductGrid";
import { SearchBar } from "@/components/SearchBar";
import { getApiUrl } from "@/lib/api";

async function fetchJson<T>(path: string): Promise<T> {
  const res = await fetch(`${getApiUrl()}${path}`, {
    cache: "no-store",
  });
  if (!res.ok) throw new Error(`Failed to load ${path}`);
  return res.json() as Promise<T>;
}

export default async function HomePage({
  searchParams,
}: {
  searchParams: Promise<{ q?: string; category?: string }>;
}) {
  const sp = await searchParams;
  const params = new URLSearchParams();
  if (sp.q) params.set("q", sp.q);
  if (sp.category) params.set("category", sp.category);
  const qs = params.toString();

  let products: Product[] = [];
  let categories: Category[] = [];
  let error: string | null = null;

  try {
    [products, categories] = await Promise.all([
      fetchJson<Product[]>(`/products${qs ? `?${qs}` : ""}`),
      fetchJson<Category[]>("/categories"),
    ]);
  } catch {
    error = "서버에 연결할 수 없습니다. web-server와 controller를 실행해주세요.";
  }

  return (
    <>
      <HeroSlider products={products} />
      <Suspense fallback={null}>
        <SearchBar categories={categories} />
      </Suspense>
      <section className="container">
        <div className="section-head">
          <h2>NEW COLLECTION</h2>
          <p className="muted">관리자가 등록한 상품</p>
        </div>
        {error ? <p className="error">{error}</p> : <ProductGrid products={products} />}
      </section>
      <section className="footer-banner">
        <div className="container">
          <h2>BUILT FOR EVERYDAY SHOPPING</h2>
          <p>무인 장보기 마트 데모 · SmartShop</p>
        </div>
      </section>
    </>
  );
}
