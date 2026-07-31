import type { GuestCartItem } from "@smartshop/shared";

const KEY = "smartshop.guestCart";

export function getGuestCart(): GuestCartItem[] {
  if (typeof window === "undefined") return [];
  try {
    const raw = localStorage.getItem(KEY);
    if (!raw) return [];
    const parsed = JSON.parse(raw) as GuestCartItem[];
    return Array.isArray(parsed) ? parsed : [];
  } catch {
    return [];
  }
}

export function setGuestCart(items: GuestCartItem[]) {
  localStorage.setItem(KEY, JSON.stringify(items));
  window.dispatchEvent(new Event("smartshop-cart"));
}

export function clearGuestCart() {
  localStorage.removeItem(KEY);
  window.dispatchEvent(new Event("smartshop-cart"));
}

export function addGuestItem(productId: number, quantity = 1) {
  const items = getGuestCart();
  const existing = items.find((i) => i.productId === productId);
  if (existing) existing.quantity += quantity;
  else items.push({ productId, quantity });
  setGuestCart(items);
}

export function updateGuestItem(productId: number, quantity: number) {
  let items = getGuestCart();
  if (quantity <= 0) items = items.filter((i) => i.productId !== productId);
  else {
    const existing = items.find((i) => i.productId === productId);
    if (existing) existing.quantity = quantity;
    else items.push({ productId, quantity });
  }
  setGuestCart(items);
}

export function guestCount(): number {
  return getGuestCart().reduce((sum, i) => sum + i.quantity, 0);
}
