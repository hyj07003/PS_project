export type UserRole = "customer" | "admin";
export type UserStatus = "active" | "disabled";

export type OrderStatus =
  | "CREATED"
  | "ASSIGNED"
  | "PICKING"
  | "CHECKOUT"
  | "PACKING"
  | "COMPLETED"
  | "CANCELLED"
  | "FAILED";

export type DeviceType = "cart" | "station";
export type DeviceStatus = "idle" | "busy" | "error" | "offline";

export interface UserPublic {
  id: number;
  email: string;
  name: string;
  role: UserRole;
  status: UserStatus;
  createdAt: string;
}

export interface Category {
  id: number;
  code: string;
  name: string;
  sortOrder: number;
  isActive: boolean;
}

export interface Product {
  id: number;
  categoryId: number;
  categoryCode?: string;
  categoryName?: string;
  name: string;
  slug: string;
  description: string | null;
  price: number;
  stock: number;
  imageFullUrl: string | null;
  imageZoomUrl: string | null;
  isFeatured: boolean;
  isActive: boolean;
  createdBy: number | null;
  createdAt: string;
  updatedAt: string;
}

export interface CartItem {
  id: number;
  productId: number;
  quantity: number;
  product?: Product;
}

export interface Cart {
  id: number;
  userId: number;
  items: CartItem[];
  updatedAt: string;
}

export interface GuestCartItem {
  productId: number;
  quantity: number;
}

export interface OrderItem {
  id: number;
  productId: number;
  productName: string;
  unitPrice: number;
  quantity: number;
}

export interface Order {
  id: number;
  userId: number | null;
  status: OrderStatus;
  totalPrice: number;
  items: OrderItem[];
  createdAt: string;
  updatedAt: string;
}

export interface CreateProductInput {
  categoryId: number;
  name: string;
  slug?: string;
  description?: string;
  price: number;
  stock?: number;
  imageFullUrl?: string;
  imageZoomUrl?: string;
  isFeatured?: boolean;
  isActive?: boolean;
}

export interface UpdateProductInput extends Partial<CreateProductInput> {}

export interface AuthRegisterInput {
  email: string;
  password: string;
  name: string;
}

export interface AuthLoginInput {
  email: string;
  password: string;
}

export interface AuthResponse {
  token: string;
  user: UserPublic;
}

export const CATEGORY_SEEDS = [
  { code: "fresh", name: "신선식품", sortOrder: 1 },
  { code: "dairy", name: "유제품", sortOrder: 2 },
  { code: "beverage", name: "음료", sortOrder: 3 },
  { code: "snack", name: "스낵/과자", sortOrder: 4 },
  { code: "ready", name: "즉석식품", sortOrder: 5 },
  { code: "household", name: "생활용품", sortOrder: 6 },
  { code: "kitchen", name: "주방/세제", sortOrder: 7 },
  { code: "health", name: "헬스/뷰티", sortOrder: 8 },
] as const;

export function slugify(input: string): string {
  return input
    .toLowerCase()
    .trim()
    .replace(/[^\w\uac00-\ud7a3]+/g, "-")
    .replace(/^-+|-+$/g, "")
    .slice(0, 80) || `item-${Date.now()}`;
}

export function formatPriceKrw(price: number): string {
  return new Intl.NumberFormat("ko-KR", {
    style: "currency",
    currency: "KRW",
    maximumFractionDigits: 0,
  }).format(price);
}
