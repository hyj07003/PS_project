import type { Product } from "@smartshop/shared";

type ProductRow = {
  id: number;
  category_id: number;
  category_code?: string;
  category_name?: string;
  name: string;
  slug: string;
  description: string | null;
  price: number;
  stock: number;
  image_full_url: string | null;
  image_zoom_url: string | null;
  is_featured: number;
  is_active: number;
  created_by: number | null;
  created_at: string;
  updated_at: string;
};

/** Store/serve local uploads as path-only so clients can resolve LAN host. */
export function toStoredMediaUrl(url: string | null | undefined): string | null {
  if (!url || !url.trim()) return null;
  const trimmed = url.trim();
  if (trimmed.startsWith("/")) return trimmed;
  try {
    const parsed = new URL(trimmed);
    if (
      parsed.hostname === "127.0.0.1" ||
      parsed.hostname === "localhost"
    ) {
      return `${parsed.pathname}${parsed.search}`;
    }
  } catch {
    // keep as-is
  }
  return trimmed;
}

function shouldReuseFullAsZoom(
  full: string | null,
  zoom: string | null,
): boolean {
  if (!full) return false;
  if (!zoom) return true;
  if (full === zoom) return false;
  // Full replaced with upload, but zoom still seed placeholder / old remote
  if (full.startsWith("/uploads/") && /placehold\.co/i.test(zoom)) return true;
  if (full.startsWith("/uploads/") && /^https?:\/\//i.test(zoom) && !zoom.includes("/uploads/")) {
    return true;
  }
  return false;
}

/**
 * One image → both slots.
 * Local upload full + leftover placeholder zoom → zoom becomes full.
 */
export function resolveProductImages(
  full: string | null | undefined,
  zoom: string | null | undefined,
  options?: { preferFullForZoom?: boolean },
): { full: string | null; zoom: string | null } {
  let imageFull = toStoredMediaUrl(full);
  let imageZoom = toStoredMediaUrl(zoom);

  if (options?.preferFullForZoom && imageFull) {
    imageZoom = imageFull;
  } else if (shouldReuseFullAsZoom(imageFull, imageZoom)) {
    imageZoom = imageFull;
  } else if (imageZoom && !imageFull) {
    imageFull = imageZoom;
  }

  return { full: imageFull, zoom: imageZoom };
}

export function mapProduct(row: ProductRow): Product {
  const { full, zoom } = resolveProductImages(
    row.image_full_url,
    row.image_zoom_url,
  );
  return {
    id: row.id,
    categoryId: row.category_id,
    categoryCode: row.category_code,
    categoryName: row.category_name,
    name: row.name,
    slug: row.slug,
    description: row.description,
    price: row.price,
    stock: row.stock,
    imageFullUrl: full,
    imageZoomUrl: zoom,
    isFeatured: !!row.is_featured,
    isActive: !!row.is_active,
    createdBy: row.created_by,
    createdAt: row.created_at,
    updatedAt: row.updated_at,
  };
}

export function nowIso(): string {
  return new Date().toISOString();
}
