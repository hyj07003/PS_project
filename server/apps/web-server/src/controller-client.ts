import * as fs from "fs";
import * as path from "path";

export function loadEnv() {
  const candidates = [
    path.resolve(process.cwd(), ".env"),
    path.resolve(process.cwd(), "../../.env"),
  ];
  for (const file of candidates) {
    if (!fs.existsSync(file)) continue;
    const text = fs.readFileSync(file, "utf8");
    for (const line of text.split(/\r?\n/)) {
      const trimmed = line.trim();
      if (!trimmed || trimmed.startsWith("#")) continue;
      const idx = trimmed.indexOf("=");
      if (idx < 0) continue;
      const key = trimmed.slice(0, idx).trim();
      const value = trimmed.slice(idx + 1).trim();
      if (!(key in process.env)) process.env[key] = value;
    }
  }
}

export function controllerUrl(): string {
  return process.env.CONTROLLER_URL || "http://127.0.0.1:4100";
}

export async function controllerFetch(
  pathname: string,
  init?: RequestInit,
): Promise<Response> {
  const url = `${controllerUrl()}${pathname}`;
  return fetch(url, {
    ...init,
    headers: {
      "Content-Type": "application/json",
      ...(init?.headers || {}),
    },
  });
}

export async function controllerJson<T>(
  pathname: string,
  init?: RequestInit,
): Promise<T> {
  const res = await controllerFetch(pathname, init);
  const text = await res.text();
  let data: unknown = null;
  try {
    data = text ? JSON.parse(text) : null;
  } catch {
    data = { message: text };
  }
  if (!res.ok) {
    const message =
      typeof data === "object" &&
      data &&
      "message" in data &&
      (data as { message: unknown }).message
        ? String((data as { message: unknown }).message)
        : res.statusText;
    const err = new Error(message) as Error & { status: number };
    err.status = res.status;
    throw err;
  }
  return data as T;
}
