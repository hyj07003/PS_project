const DEFAULT_API_PORT = process.env.NEXT_PUBLIC_API_PORT || "4000";

/** Browser: same host as the page (LAN IP works on phone). Server: localhost. */
export function getApiUrl(): string {
  if (typeof window !== "undefined") {
    return `${window.location.protocol}//${window.location.hostname}:${DEFAULT_API_PORT}`;
  }
  return (
    process.env.NEXT_PUBLIC_API_URL ||
    process.env.WEB_SERVER_URL ||
    `http://127.0.0.1:${DEFAULT_API_PORT}`
  );
}

/**
 * Media URLs must be identical on server + client (avoid hydration mismatch).
 * Local uploads stay path-only (`/uploads/...`) and are proxied by Next rewrites.
 */
export function resolveMediaUrl(url: string | null | undefined): string {
  if (!url) return "";
  if (url.startsWith("/")) {
    return url;
  }
  try {
    const parsed = new URL(url);
    if (
      parsed.hostname === "127.0.0.1" ||
      parsed.hostname === "localhost"
    ) {
      return `${parsed.pathname}${parsed.search}`;
    }
  } catch {
    // keep original
  }
  return url;
}

export const API_URL = getApiUrl();

export function getToken(): string | null {
  if (typeof window === "undefined") return null;
  return localStorage.getItem("smartshop.token");
}

export function setToken(token: string | null) {
  if (typeof window === "undefined") return;
  if (token) localStorage.setItem("smartshop.token", token);
  else localStorage.removeItem("smartshop.token");
}

export async function api<T>(
  path: string,
  init?: RequestInit & { auth?: boolean },
): Promise<T> {
  const headers = new Headers(init?.headers);
  if (!headers.has("Content-Type") && init?.body) {
    headers.set("Content-Type", "application/json");
  }
  if (init?.auth !== false) {
    const token = getToken();
    if (token) headers.set("Authorization", `Bearer ${token}`);
  }
  const res = await fetch(`${getApiUrl()}${path}`, { ...init, headers });
  const text = await res.text();
  let data: unknown = null;
  try {
    data = text ? JSON.parse(text) : null;
  } catch {
    data = { message: text };
  }
  if (!res.ok) {
    const message =
      typeof data === "object" && data && "message" in data
        ? String((data as { message: unknown }).message)
        : res.statusText;
    throw new Error(Array.isArray(message) ? message.join(", ") : message);
  }
  return data as T;
}
