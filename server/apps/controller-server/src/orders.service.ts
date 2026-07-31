import {
  BadRequestException,
  Inject,
  Injectable,
  NotFoundException,
} from "@nestjs/common";
import type { Order, OrderStatus } from "@smartshop/shared";
import {
  MockAiAdapter,
  MockCartAdapter,
  MockStationAdapter,
  ORDER_FLOW,
} from "./adapters/ports";
import { CartsService } from "./carts.service";
import { DatabaseService } from "./database.service";
import { nowIso } from "./db/mappers";

@Injectable()
export class OrdersService {
  private readonly cartPort = new MockCartAdapter();
  private readonly stationPort = new MockStationAdapter();
  private readonly aiPort = new MockAiAdapter();

  constructor(
    @Inject(DatabaseService) private readonly db: DatabaseService,
    @Inject(CartsService) private readonly carts: CartsService,
  ) {}

  getById(id: number): Order {
    const order = this.db.connection
      .prepare(`SELECT * FROM orders WHERE id = ?`)
      .get(id) as
      | {
          id: number;
          user_id: number | null;
          status: OrderStatus;
          total_price: number;
          created_at: string;
          updated_at: string;
        }
      | undefined;
    if (!order) throw new NotFoundException("order not found");

    const items = this.db.connection
      .prepare(`SELECT * FROM order_items WHERE order_id = ?`)
      .all(id) as Array<{
      id: number;
      product_id: number;
      product_name: string;
      unit_price: number;
      quantity: number;
    }>;

    return {
      id: order.id,
      userId: order.user_id,
      status: order.status,
      totalPrice: order.total_price,
      createdAt: order.created_at,
      updatedAt: order.updated_at,
      items: items.map((i) => ({
        id: i.id,
        productId: i.product_id,
        productName: i.product_name,
        unitPrice: i.unit_price,
        quantity: i.quantity,
      })),
    };
  }

  async createFromCart(userId: number): Promise<Order> {
    const cart = this.carts.getCart(userId);
    if (!cart.items.length) {
      throw new BadRequestException("cart is empty");
    }

    const ts = nowIso();
    const total = cart.items.reduce(
      (sum, item) => sum + (item.product?.price ?? 0) * item.quantity,
      0,
    );

    this.db.connection.exec("BEGIN");
    let orderId = 0;
    let missionId = 0;
    try {
      const orderResult = this.db.connection
        .prepare(
          `INSERT INTO orders (user_id, status, total_price, created_at, updated_at)
           VALUES (?, 'CREATED', ?, ?, ?)`,
        )
        .run(userId, total, ts, ts);
      orderId = Number(orderResult.lastInsertRowid);

      const insertItem = this.db.connection.prepare(
        `INSERT INTO order_items (order_id, product_id, product_name, unit_price, quantity)
         VALUES (?, ?, ?, ?, ?)`,
      );
      for (const item of cart.items) {
        insertItem.run(
          orderId,
          item.productId,
          item.product!.name,
          item.product!.price,
          item.quantity,
        );
      }

      const missionResult = this.db.connection
        .prepare(
          `INSERT INTO missions (order_id, device_id, status, created_at)
           VALUES (?, NULL, 'CREATED', ?)`,
        )
        .run(orderId, ts);
      missionId = Number(missionResult.lastInsertRowid);
      this.db.connection
        .prepare(
          `INSERT INTO mission_events (mission_id, from_status, to_status, note, created_at)
           VALUES (?, NULL, 'CREATED', 'order created', ?)`,
        )
        .run(missionId, ts);
      this.db.connection.exec("COMMIT");
    } catch (err) {
      this.db.connection.exec("ROLLBACK");
      throw err;
    }

    this.carts.clear(userId);

    void this.runMockPipeline(orderId, missionId);
    return this.getById(orderId);
  }

  private async runMockPipeline(orderId: number, missionId: number) {
    try {
      await this.aiPort.requestPickPlan(orderId);
      const assigned = await this.cartPort.assignCart(orderId);
      if (assigned) {
        const device = this.db.connection
          .prepare(`SELECT id FROM devices WHERE code = ?`)
          .get(assigned.deviceCode) as { id: number } | undefined;
        if (device) {
          this.db.connection
            .prepare(`UPDATE missions SET device_id = ? WHERE id = ?`)
            .run(device.id, missionId);
          this.db.connection
            .prepare(`UPDATE devices SET status = 'busy' WHERE id = ?`)
            .run(device.id);
        }
        await this.cartPort.navigate(assigned.deviceCode, "aisle-a");
      }
      this.setStatus(orderId, missionId, "ASSIGNED");
      await this.stationPort.startPicking(orderId);
      this.setStatus(orderId, missionId, "PICKING");
      await this.stationPort.checkout(orderId);
      this.setStatus(orderId, missionId, "CHECKOUT");
      await this.stationPort.pack(orderId);
      this.setStatus(orderId, missionId, "PACKING");
      this.setStatus(orderId, missionId, "COMPLETED");

      const mission = this.db.connection
        .prepare(`SELECT device_id FROM missions WHERE id = ?`)
        .get(missionId) as { device_id: number | null };
      if (mission.device_id) {
        this.db.connection
          .prepare(`UPDATE devices SET status = 'idle' WHERE id = ?`)
          .run(mission.device_id);
      }
    } catch {
      this.setStatus(orderId, missionId, "FAILED");
    }
  }

  private setStatus(orderId: number, missionId: number, status: OrderStatus) {
    const current = this.db.connection
      .prepare(`SELECT status FROM orders WHERE id = ?`)
      .get(orderId) as { status: OrderStatus };
    const ts = nowIso();
    this.db.connection
      .prepare(`UPDATE orders SET status = ?, updated_at = ? WHERE id = ?`)
      .run(status, ts, orderId);
    this.db.connection
      .prepare(`UPDATE missions SET status = ? WHERE id = ?`)
      .run(status, missionId);
    this.db.connection
      .prepare(
        `INSERT INTO mission_events (mission_id, from_status, to_status, note, created_at)
         VALUES (?, ?, ?, ?, ?)`,
      )
      .run(missionId, current.status, status, `flow:${ORDER_FLOW.indexOf(status)}`, ts);
  }

  listDevices() {
    return this.db.connection.prepare(`SELECT * FROM devices`).all();
  }
}
