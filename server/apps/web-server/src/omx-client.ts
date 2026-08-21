/**
 * OMX robot-arm HTTP client (BFF → OMX PC :8080).
 * OMX_URL from server/.env, e.g. http://192.168.129.50:8080
 */

export function omxUrl(): string | null {
  const raw = (process.env.OMX_URL || "").trim().replace(/\/$/, "");
  return raw || null;
}

export function omxConfigured(): boolean {
  return Boolean(omxUrl());
}

export async function omxJson<T = Record<string, unknown>>(
  pathname: string,
  init?: RequestInit,
): Promise<T> {
  const base = omxUrl();
  if (!base) {
    const err = new Error("OMX_URL is not configured") as Error & {
      status: number;
    };
    err.status = 503;
    throw err;
  }
  const timeoutMs = Math.round(
    Number(process.env.OMX_CONNECT_TIMEOUT_SEC || "5") * 1000,
  );
  const ctrl = new AbortController();
  const timer = setTimeout(() => ctrl.abort(), Math.max(1000, timeoutMs));
  try {
    const res = await fetch(`${base}${pathname}`, {
      ...init,
      signal: ctrl.signal,
    });
    const text = await res.text();
    let data: unknown = {};
    try {
      data = text ? JSON.parse(text) : {};
    } catch {
      data = { message: text.slice(0, 200) };
    }
    if (!res.ok) {
      const message =
        data && typeof data === "object" && "message" in data
          ? String((data as { message?: unknown }).message || "")
          : res.statusText;
      const err = new Error(message || "omx upstream error") as Error & {
        status: number;
      };
      err.status = res.status;
      throw err;
    }
    return data as T;
  } finally {
    clearTimeout(timer);
  }
}

/** Open OMX MJPEG / JPEG for streaming proxy (caller pipes body). */
export async function omxFetch(
  pathname: string,
): Promise<Response> {
  const base = omxUrl();
  if (!base) {
    const err = new Error("OMX_URL is not configured") as Error & {
      status: number;
    };
    err.status = 503;
    throw err;
  }
  const res = await fetch(`${base}${pathname}`);
  if (!res.ok) {
    const err = new Error(
      `omx ${pathname} → ${res.status}`,
    ) as Error & { status: number };
    err.status = res.status;
    throw err;
  }
  return res;
}
