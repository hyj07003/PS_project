import type { OrderStatus } from "@smartshop/shared";

export interface CartPort {
  assignCart(orderId: number): Promise<{ deviceCode: string } | null>;
  navigate(deviceCode: string, waypoint: string): Promise<"ARRIVED" | "FAILED">;
}

export interface StationPort {
  startPicking(orderId: number): Promise<"DONE" | "FAILED">;
  checkout(orderId: number): Promise<"DONE" | "FAILED">;
  pack(orderId: number): Promise<"DONE" | "FAILED">;
}

export interface AiPort {
  requestPickPlan(orderId: number): Promise<{ waypoints: string[] }>;
}

function delay(ms: number): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

export class MockCartAdapter implements CartPort {
  private next = 0;

  async assignCart(): Promise<{ deviceCode: string } | null> {
    await delay(200);
    this.next = (this.next % 2) + 1;
    return { deviceCode: `cart-${this.next}` };
  }

  async navigate(): Promise<"ARRIVED" | "FAILED"> {
    await delay(800);
    return "ARRIVED";
  }
}

export class MockStationAdapter implements StationPort {
  async startPicking(): Promise<"DONE" | "FAILED"> {
    await delay(600);
    return "DONE";
  }

  async checkout(): Promise<"DONE" | "FAILED"> {
    await delay(400);
    return "DONE";
  }

  async pack(): Promise<"DONE" | "FAILED"> {
    await delay(500);
    return "DONE";
  }
}

export class MockAiAdapter implements AiPort {
  async requestPickPlan(): Promise<{ waypoints: string[] }> {
    await delay(150);
    return { waypoints: ["aisle-a", "aisle-b", "checkout"] };
  }
}

export const ORDER_FLOW: OrderStatus[] = [
  "CREATED",
  "ASSIGNED",
  "PICKING",
  "CHECKOUT",
  "PACKING",
  "COMPLETED",
];
