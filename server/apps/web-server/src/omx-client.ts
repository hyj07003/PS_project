/**
 * OMX HTTP clients (BFF → OMX PC).
 * 진열대: OMX_URL :8080  /  계산대: PACK_URL :8081
 */

async function omxHttpJson<T>(
  base: string,
  connectTimeoutSec: string,
  pathname: string,
  missingLabel: string,
  init?: RequestInit,
): Promise<T> {
  if (!base) {
    const err = new Error(`${missingLabel} is not configured`) as Error & {
      status: number;
    };
    err.status = 503;
    throw err;
  }
  const timeoutMs = Math.round(Number(connectTimeoutSec || "5") * 1000);
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

async function omxHttpFetch(
  base: string,
  pathname: string,
  missingLabel: string,
): Promise<Response> {
  if (!base) {
    const err = new Error(`${missingLabel} is not configured`) as Error & {
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

/** 진열대 OMX (:8080) */
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
  return omxHttpJson(
    omxUrl() || "",
    process.env.OMX_CONNECT_TIMEOUT_SEC || "5",
    pathname,
    "OMX_URL",
    init,
  );
}

export async function omxFetch(pathname: string): Promise<Response> {
  return omxHttpFetch(omxUrl() || "", pathname, "OMX_URL");
}

/** 계산대 OMX (:8081) */
export function packUrl(): string | null {
  const raw = (process.env.PACK_URL || "").trim().replace(/\/$/, "");
  return raw || null;
}

export function packConfigured(): boolean {
  return Boolean(packUrl());
}

export async function packJson<T = Record<string, unknown>>(
  pathname: string,
  init?: RequestInit,
): Promise<T> {
  return omxHttpJson(
    packUrl() || "",
    process.env.PACK_CONNECT_TIMEOUT_SEC || "5",
    pathname,
    "PACK_URL",
    init,
  );
}

export async function packFetch(pathname: string): Promise<Response> {
  return omxHttpFetch(packUrl() || "", pathname, "PACK_URL");
}
