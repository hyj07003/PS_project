import {
  BadRequestException,
  Inject,
  Injectable,
  NotFoundException,
} from "@nestjs/common";
import type { Category, CreateProductInput, UpdateProductInput } from "@smartshop/shared";
import { slugify } from "@smartshop/shared";
import { DatabaseService } from "./database.service";
import { mapProduct, nowIso, resolveProductImages } from "./db/mappers";

const PRODUCT_SELECT = `
  SELECT p.*, c.code AS category_code, c.name AS category_name
  FROM products p
  JOIN categories c ON c.id = p.category_id
`;

@Injectable()
export class ProductsService {
  constructor(@Inject(DatabaseService) private readonly db: DatabaseService) {}

  listCategories(): Category[] {
    const rows = this.db.connection
      .prepare(
        `SELECT id, code, name, sort_order, is_active FROM categories
         WHERE is_active = 1 ORDER BY sort_order ASC`,
      )
      .all() as Array<{
      id: number;
      code: string;
      name: string;
      sort_order: number;
      is_active: number;
    }>;
    return rows.map((r) => ({
      id: r.id,
      code: r.code,
      name: r.name,
      sortOrder: r.sort_order,
      isActive: !!r.is_active,
    }));
  }

  list(params: {
    q?: string;
    category?: string;
    featured?: boolean;
    activeOnly?: boolean;
    includeInactive?: boolean;
  }) {
    const where: string[] = [];
    const args: unknown[] = [];

    if (params.activeOnly || !params.includeInactive) {
      where.push("p.is_active = 1");
    }
    if (params.featured) {
      where.push("p.is_featured = 1");
    }
    if (params.category) {
      where.push("(c.code = ? OR c.name = ?)");
      args.push(params.category, params.category);
    }
    if (params.q) {
      where.push("(p.name LIKE ? OR IFNULL(p.description, '') LIKE ?)");
      const like = `%${params.q}%`;
      args.push(like, like);
    }

    const sql = `${PRODUCT_SELECT}
      ${where.length ? `WHERE ${where.join(" AND ")}` : ""}
      ORDER BY p.is_featured DESC, p.id DESC`;

    const rows = this.db.connection.prepare(sql).all(...args);
    return rows.map((r) => mapProduct(r as never));
  }

  getById(id: number, activeOnly = false) {
    const row = this.db.connection
      .prepare(
        `${PRODUCT_SELECT} WHERE p.id = ? ${activeOnly ? "AND p.is_active = 1" : ""}`,
      )
      .get(id);
    if (!row) throw new NotFoundException("product not found");
    return mapProduct(row as never);
  }

  create(input: CreateProductInput, createdBy?: number) {
    if (!input.name || input.price == null || !input.categoryId) {
      throw new BadRequestException("name, price, categoryId required");
    }
    const category = this.db.connection
      .prepare("SELECT id FROM categories WHERE id = ?")
      .get(input.categoryId);
    if (!category) throw new BadRequestException("invalid categoryId");

    const ts = nowIso();
    let slug = input.slug?.trim() || slugify(input.name);
    const exists = this.db.connection
      .prepare("SELECT id FROM products WHERE slug = ?")
      .get(slug);
    if (exists) slug = `${slug}-${Date.now()}`;

    const images = resolveProductImages(
      input.imageFullUrl,
      input.imageZoomUrl,
    );

    const result = this.db.connection
      .prepare(
        `INSERT INTO products (
          category_id, name, slug, description, price, stock,
          image_full_url, image_zoom_url, is_featured, is_active,
          created_by, created_at, updated_at
        ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)`,
      )
      .run(
        input.categoryId,
        input.name,
        slug,
        input.description ?? null,
        input.price,
        input.stock ?? 0,
        images.full,
        images.zoom,
        input.isFeatured ? 1 : 0,
        input.isActive === false ? 0 : 1,
        createdBy ?? null,
        ts,
        ts,
      );

    return this.getById(Number(result.lastInsertRowid));
  }

  update(id: number, input: UpdateProductInput) {
    const current = this.getById(id);
    const ts = nowIso();
    const slug =
      input.slug?.trim() ||
      (input.name ? slugify(input.name) : current.slug);

    const nextFull =
      input.imageFullUrl !== undefined
        ? input.imageFullUrl
        : current.imageFullUrl;
    const nextZoom =
      input.imageZoomUrl !== undefined
        ? input.imageZoomUrl
        : current.imageZoomUrl;
    const images = resolveProductImages(nextFull, nextZoom);

    this.db.connection
      .prepare(
        `UPDATE products SET
          category_id = ?,
          name = ?,
          slug = ?,
          description = ?,
          price = ?,
          stock = ?,
          image_full_url = ?,
          image_zoom_url = ?,
          is_featured = ?,
          is_active = ?,
          updated_at = ?
         WHERE id = ?`,
      )
      .run(
        input.categoryId ?? current.categoryId,
        input.name ?? current.name,
        slug,
        input.description !== undefined
          ? input.description
          : current.description,
        input.price ?? current.price,
        input.stock ?? current.stock,
        images.full,
        images.zoom,
        input.isFeatured !== undefined
          ? input.isFeatured
            ? 1
            : 0
          : current.isFeatured
            ? 1
            : 0,
        input.isActive !== undefined
          ? input.isActive
            ? 1
            : 0
          : current.isActive
            ? 1
            : 0,
        ts,
        id,
      );

    return this.getById(id);
  }

  remove(id: number) {
    this.getById(id);
    this.db.connection.prepare("DELETE FROM products WHERE id = ?").run(id);
    return { ok: true };
  }
}
