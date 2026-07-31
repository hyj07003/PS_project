"use client";

import type { AuthResponse, Cart, GuestCartItem, UserPublic } from "@smartshop/shared";
import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useState,
  type ReactNode,
} from "react";
import { api, getToken, setToken } from "@/lib/api";
import {
  addGuestItem,
  clearGuestCart,
  getGuestCart,
  guestCount,
  updateGuestItem,
} from "@/lib/guest-cart";

type AuthContextValue = {
  user: UserPublic | null;
  loading: boolean;
  cartCount: number;
  login: (email: string, password: string) => Promise<void>;
  register: (email: string, password: string, name: string) => Promise<void>;
  logout: () => void;
  refreshCartCount: () => Promise<void>;
  addToCart: (productId: number, quantity?: number) => Promise<void>;
};

const AuthContext = createContext<AuthContextValue | null>(null);

export function AuthProvider({ children }: { children: ReactNode }) {
  const [user, setUser] = useState<UserPublic | null>(null);
  const [loading, setLoading] = useState(true);
  const [cartCount, setCartCount] = useState(0);

  const refreshCartCount = useCallback(async () => {
    const token = getToken();
    if (!token) {
      setCartCount(guestCount());
      return;
    }
    try {
      const cart = await api<Cart>("/cart");
      setCartCount(cart.items.reduce((s, i) => s + i.quantity, 0));
    } catch {
      setCartCount(guestCount());
    }
  }, []);

  const mergeGuest = useCallback(async () => {
    const items = getGuestCart();
    if (!items.length) return;
    await api<Cart>("/cart/merge", {
      method: "POST",
      body: JSON.stringify({ items }),
    });
    clearGuestCart();
  }, []);

  useEffect(() => {
    const boot = async () => {
      const token = getToken();
      if (!token) {
        setLoading(false);
        setCartCount(guestCount());
        return;
      }
      try {
        const me = await api<UserPublic>("/auth/me");
        setUser(me);
        await mergeGuest();
        await refreshCartCount();
      } catch {
        setToken(null);
        setUser(null);
        setCartCount(guestCount());
      } finally {
        setLoading(false);
      }
    };
    void boot();

    const onCart = () => {
      if (!getToken()) setCartCount(guestCount());
    };
    window.addEventListener("smartshop-cart", onCart);
    return () => window.removeEventListener("smartshop-cart", onCart);
  }, [mergeGuest, refreshCartCount]);

  const login = useCallback(
    async (email: string, password: string) => {
      const res = await api<AuthResponse>("/auth/login", {
        method: "POST",
        body: JSON.stringify({ email, password }),
        auth: false,
      });
      setToken(res.token);
      setUser(res.user);
      await mergeGuest();
      await refreshCartCount();
    },
    [mergeGuest, refreshCartCount],
  );

  const register = useCallback(
    async (email: string, password: string, name: string) => {
      const res = await api<AuthResponse>("/auth/register", {
        method: "POST",
        body: JSON.stringify({ email, password, name }),
        auth: false,
      });
      setToken(res.token);
      setUser(res.user);
      await mergeGuest();
      await refreshCartCount();
    },
    [mergeGuest, refreshCartCount],
  );

  const logout = useCallback(() => {
    setToken(null);
    setUser(null);
    setCartCount(guestCount());
  }, []);

  const addToCart = useCallback(
    async (productId: number, quantity = 1) => {
      if (user) {
        await api("/cart/items", {
          method: "POST",
          body: JSON.stringify({ productId, quantity }),
        });
        await refreshCartCount();
      } else {
        addGuestItem(productId, quantity);
        setCartCount(guestCount());
      }
    },
    [refreshCartCount, user],
  );

  const value = useMemo(
    () => ({
      user,
      loading,
      cartCount,
      login,
      register,
      logout,
      refreshCartCount,
      addToCart,
    }),
    [
      user,
      loading,
      cartCount,
      login,
      register,
      logout,
      refreshCartCount,
      addToCart,
    ],
  );

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

export function useAuth() {
  const ctx = useContext(AuthContext);
  if (!ctx) throw new Error("useAuth must be used within AuthProvider");
  return ctx;
}

export async function resolveGuestCartProducts() {
  const items = getGuestCart();
  return items as GuestCartItem[];
}

export { updateGuestItem, getGuestCart, clearGuestCart };
