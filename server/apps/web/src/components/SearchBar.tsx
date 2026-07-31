"use client";

import { useRouter, useSearchParams } from "next/navigation";
import { FormEvent, useState } from "react";
import type { Category } from "@smartshop/shared";

export function SearchBar({ categories }: { categories: Category[] }) {
  const router = useRouter();
  const params = useSearchParams();
  const [q, setQ] = useState(params.get("q") || "");
  const [category, setCategory] = useState(params.get("category") || "");

  const onSubmit = (e: FormEvent) => {
    e.preventDefault();
    const next = new URLSearchParams();
    if (q.trim()) next.set("q", q.trim());
    if (category) next.set("category", category);
    router.push(`/?${next.toString()}`);
  };

  return (
    <form className="search-bar container" onSubmit={onSubmit}>
      <div className="search-fields">
        <input
          value={q}
          onChange={(e) => setQ(e.target.value)}
          placeholder="상품 검색"
          aria-label="상품 검색"
        />
        <select
          value={category}
          onChange={(e) => setCategory(e.target.value)}
          aria-label="카테고리"
        >
          <option value="">전체 카테고리</option>
          {categories.map((c) => (
            <option key={c.id} value={c.code}>
              {c.name}
            </option>
          ))}
        </select>
      </div>
      <button type="submit" className="btn">
        검색
      </button>
    </form>
  );
}
