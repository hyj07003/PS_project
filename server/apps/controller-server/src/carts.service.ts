import {
  BadRequestException,
  Inject,
  Injectable,
  NotFoundException,
} from "@nestjs/common";
import type { Cart, GuestCartItem } from "@smartshop/shared";
import { DatabaseService } from "./database.service";
import { mapProduct, nowIso } from "./db/mappers";
import { ProductsService } from "./products.service";

@Injectable()
export class CartsService {
  constructor(
    @Inject(DatabaseService) private readonly db: DatabaseService,
    @Inject(ProductsService) private readonly products: ProductsService,
  ) {}

  private ensureCart(userId: number): number {
    const existing = this.db.connection
      .prepare("SELECT id FROM carts WHERE user_id = ?")
      .get(userId) as { id: number } | undefined;
    if (existing) return existing.id;
    const ts = nowIso();
    const result = this.db.connection
      .prepare(`INSERT INTO carts (user_id, updated_at) VALUES (?, ?)`)
      .run(userId, ts);
    return Number(result.lastInsertRowid);
  }

  getCart(userId: number): Cart {
    const cartId = this.ensureCart(userId);
    const cart = this.db.connection
      .prepare("SELECT id, user_id, updated_at FROM carts WHERE id = ?")
      .get(cartId) as { id: number; user_id: number; updated_at: string };

    const items = this.db.connection
      .prepare(
        `SELECT ci.id, ci.product_id, ci.quantity,
                p.*, c.code AS category_code, c.name AS category_name
         FROM cart_items ci
         JOIN products p ON p.id = ci.product_id
         JOIN categories c ON c.id = p.category_id
         WHERE ci.cart_id = ?`,
      )
      .all(cartId) as Array<Record<string, unknown>>;

    return {
      id: cart.id,
      userId: cart.user_id,
      updatedAt: cart.updated_at,
      items: items.map((row) => ({
        id: row.id as number,
        productId: row.product_id as number,
        quantity: row.quantity as number,
        product: mapProduct(row as never),
      })),
    };
  }

  addItem(userId: number, productId: number, quantity = 1): Cart {
    if (quantity < 1) throw new BadRequestException("quantity must be >= 1");
    this.products.getById(productId, true);
    const cartId = this.ensureCart(userId);
    const existing = this.db.connection
      .prepare(
        `SELECT id, quantity FROM cart_items WHERE cart_id = ? AND product_id = ?`,
      )
      .get(cartId, productId) as { id: number; quantity: number } | undefined;

    if (existing) {
      this.db.connection
        .prepare(`UPDATE cart_items SET quantity = ? WHERE id = ?`)
        .run(existing.quantity + quantity, existing.id);
    } else {
      this.db.connection
        .prepare(
          `INSERT INTO cart_items (cart_id, product_id, quantity) VALUES (?, ?, ?)`,
        )
        .run(cartId, productId, quantity);
    }
    this.touch(cartId);
    return this.getCart(userId);
  }

  updateItem(userId: number, productId: number, quantity: number): Cart {
    const cartId = this.ensureCart(userId);
    if (quantity <= 0) {
      this.db.connection
        .prepare(
          `DELETE FROM cart_items WHERE cart_id = ? AND product_id = ?`,
        )
        .run(cartId, productId);
    } else {
      const result = this.db.connection
        .prepare(
          `UPDATE cart_items SET quantity = ? WHERE cart_id = ? AND product_id = ?`,
        )
        .run(quantity, cartId, productId);
      if (result.changes === 0) {
        throw new NotFoundException("cart item not found");
      }
    }
    this.touch(cartId);
    return this.getCart(userId);
  }

  removeItem(userId: number, productId: number): Cart {
    return this.updateItem(userId, productId, 0);
  }

  mergeGuest(userId: number, items: GuestCartItem[]): Cart {
    for (const item of items) {
      if (!item.productId || !item.quantity) continue;
      try {
        this.addItem(userId, item.productId, item.quantity);
      } catch {
        // skip invalid guest products
      }
    }
    return this.getCart(userId);
  }

  clear(userId: number): void {
    const cartId = this.ensureCart(userId);
    this.db.connection
      .prepare(`DELETE FROM cart_items WHERE cart_id = ?`)
      .run(cartId);
    this.touch(cartId);
  }

  private touch(cartId: number) {
    this.db.connection
      .prepare(`UPDATE carts SET updated_at = ? WHERE id = ?`)
      .run(nowIso(), cartId);
  }
}
