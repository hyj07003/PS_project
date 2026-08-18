export type PinkyRobotTarget = {
  id: string;
  label: string;
  url: string;
};

/**
 * PINKY_ROBOTS=cart-1=http://192.168.129.33:4200,cart-2=http://192.168.129.34:4200
 * 또는 단일 PINKY_URL (하위 호환)
 */
export function listPinkyRobots(): PinkyRobotTarget[] {
  const raw = (process.env.PINKY_ROBOTS || "").trim();
  if (raw) {
    return raw
      .split(",")
      .map((part) => part.trim())
      .filter(Boolean)
      .map((part, i) => {
        const eq = part.indexOf("=");
        if (eq > 0) {
          const id = part.slice(0, eq).trim();
          const url = part.slice(eq + 1).trim().replace(/\/$/, "");
          return { id, label: id, url };
        }
        const url = part.replace(/\/$/, "");
        const id = `robot-${i + 1}`;
        return { id, label: id, url };
      })
      .filter((r) => r.url);
  }

  const single = (process.env.PINKY_URL || "http://127.0.0.1:4200").replace(
    /\/$/,
    "",
  );
  const second = (process.env.PINKY_URL_2 || "").trim().replace(/\/$/, "");
  const robots: PinkyRobotTarget[] = [
    { id: "cart-1", label: "cart-1", url: single },
  ];
  if (second) {
    robots.push({ id: "cart-2", label: "cart-2", url: second });
  }
  return robots;
}

export function pinkyUrl(): string {
  return listPinkyRobots()[0]?.url || "http://127.0.0.1:4200";
}

export function resolvePinkyUrl(robotId?: string): string {
  const robots = listPinkyRobots();
  if (!robotId) return robots[0]?.url || pinkyUrl();
  const found = robots.find((r) => r.id === robotId);
  if (!found) {
    const err = new Error(`unknown robot: ${robotId}`) as Error & {
      status: number;
    };
    err.status = 404;
    throw err;
  }
  return found.url;
}

/** cart-1→S1, cart-2→S2 (controller waypoints.py 와 동일). */
export function homePoseForDevice(deviceCode: string): {
  x: number;
  y: number;
  yaw: number;
} {
  const code = (deviceCode || "cart-1").trim().toLowerCase();
  if (code === "cart-2" || code === "cart2" || code === "2") {
    return {
      x: 0.04742698442363813,
      y: -0.20226078567130157,
      yaw: -0.004704072590981645,
    };
  }
  return {
    x: 0.009931882239292611,
    y: 0.021114122581406713,
    yaw: 0.01045265830576832,
  };
}

export async function pinkyFetch(
  pathname: string,
  init?: RequestInit,
  baseUrl?: string,
): Promise<Response> {
  const url = `${(baseUrl || pinkyUrl()).replace(/\/$/, "")}${pathname}`;
  return fetch(url, {
    ...init,
    headers: {
      "Content-Type": "application/json",
      ...(init?.headers || {}),
    },
  });
}

export async function pinkyJson<T>(
  pathname: string,
  init?: RequestInit,
  baseUrl?: string,
): Promise<T> {
  const res = await pinkyFetch(pathname, init, baseUrl);
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
    const err = new Error(message || "pinky upstream error") as Error & {
      status: number;
    };
    err.status = res.status;
    throw err;
  }
  return data as T;
}

/** Binary proxy (e.g. map PNG). Does not force JSON Content-Type. */
export async function pinkyBinary(
  pathname: string,
  baseUrl?: string,
): Promise<{ buffer: Buffer; contentType: string }> {
  const url = `${(baseUrl || pinkyUrl()).replace(/\/$/, "")}${pathname}`;
  const res = await fetch(url);
  if (!res.ok) {
    const err = new Error(
      `pinky binary ${pathname} → ${res.status}`,
    ) as Error & { status: number };
    err.status = res.status;
    throw err;
  }
  const ab = await res.arrayBuffer();
  return {
    buffer: Buffer.from(ab),
    contentType: res.headers.get("content-type") || "application/octet-stream",
  };
}
