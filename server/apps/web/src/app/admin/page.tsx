"use client";

import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { api } from "@/lib/api";
import { OccupancyNavMap } from "@/components/OccupancyNavMap";

type RobotHealth = {
  ok?: boolean;
  service?: string;
  backend?: string;
  deviceCode?: string;
  online?: boolean;
  sensorPublisher?: boolean;
};

type Battery = {
  percent?: number | null;
  voltage?: number | null;
  source?: string;
  isDummy?: boolean;
};

type Lidar = {
  points?: { x: number; y: number; r?: number }[];
  source?: string;
  isDummy?: boolean;
};

type Ultrasonic = {
  rangeM?: number | null;
  minRange?: number;
  maxRange?: number;
  irRaw?: number[];
  frameId?: string;
  source?: string;
  isDummy?: boolean;
};

type Snapshot = {
  deviceCode?: string;
  backend?: string;
  online?: boolean;
  battery?: Battery;
  lidar?: Lidar;
  ultrasonic?: Ultrasonic;
  pose?: { x: number; y: number; yaw: number } | null;
  navigating?: boolean;
  hasData?: {
    battery?: boolean;
    lidar?: boolean;
    ultrasonic?: boolean;
  };
  warnings?: string[];
};

type NavState = {
  pose?: { x: number; y: number; yaw: number } | null;
  navigating?: boolean;
  mapId?: string | null;
};

type Device = {
  id: number;
  code: string;
  type: string;
  status: string;
};

type OrderAssignment = {
  id: number;
  orderId: number;
  deviceCode?: string | null;
  status: string;
  currentWaypoint?: string | null;
  currentWaypointLabel?: string | null;
  order?: {
    id: number;
    status: string;
    totalPrice: number;
    items: {
      productName: string;
      unitPrice: number;
      quantity: number;
    }[];
  } | null;
};

type MissionListItem = {
  id: number;
  orderId: number;
  deviceCode?: string | null;
  status: string;
  createdAt?: string;
  currentWaypoint?: string | null;
  currentWaypointLabel?: string | null;
  order?: {
    id: number;
    status: string;
    totalPrice: number;
    createdAt?: string;
    items: {
      productName: string;
      unitPrice: number;
      quantity: number;
    }[];
  } | null;
};

type MissionEvent = {
  id: number;
  fromStatus?: string | null;
  toStatus?: string | null;
  note?: string | null;
  createdAt?: string | null;
};

type MissionDetail = MissionListItem & {
  events?: MissionEvent[];
};

type RobotMonitor = {
  id: string;
  label: string;
  url: string;
  online: boolean;
  health: RobotHealth | null;
  sensors: Snapshot | null;
  nav?: NavState | null;
  assignment?: OrderAssignment | null;
  error: string | null;
};

type RobotsResponse = {
  robots: RobotMonitor[];
  count: number;
  queueLength?: number;
};

const ROBOT_COLORS: Record<string, string> = {
  "cart-1": "#1f6f6a",
  "cart-2": "#c45c26",
};

function robotColor(id: string, index: number): string {
  if (ROBOT_COLORS[id]) return ROBOT_COLORS[id];
  const fallback = ["#1f6f6a", "#c45c26", "#3d5a80", "#8b5e34"];
  return fallback[index % fallback.length];
}

function fmt(n: number | null | undefined, digits = 2): string {
  if (n == null || Number.isNaN(n)) return "—";
  return n.toFixed(digits);
}

const DUMMY_SOURCES = new Set([
  "mock",
  "dummy",
  "fallback",
  "synthetic",
  "unavailable",
]);

function isDummySensor(
  source?: string | null,
  isDummy?: boolean,
  backend?: string | null,
): boolean {
  if (isDummy) return true;
  if ((backend || "").toLowerCase() === "mock") return true;
  return DUMMY_SOURCES.has((source || "").toLowerCase().trim());
}

const MISSION_STATUS_LABEL: Record<string, string> = {
  CREATED: "대기열",
  ASSIGNED: "할당됨",
  PICKING: "피킹 중",
  CHECKOUT: "계산대",
  PACKING: "운송대기",
  RETURNING: "대기장소 복귀 중",
  COMPLETED: "완료 (대기장소 도착)",
  FAILED: "실패",
};

function missionStatusLabel(status: string): string {
  return MISSION_STATUS_LABEL[status] || status;
}

function CartStatusPanel({
  robot,
  color,
}: {
  robot: RobotMonitor;
  color: string;
}) {
  const health = robot.health;
  const snap = robot.sensors;
  const backend = health?.backend || snap?.backend;
  const battDummy = isDummySensor(
    snap?.battery?.source,
    snap?.battery?.isDummy,
    backend,
  );
  const usDummy = isDummySensor(
    snap?.ultrasonic?.source,
    snap?.ultrasonic?.isDummy,
    backend,
  );
  const pose = robot.nav?.pose || snap?.pose || null;
  const navigating = Boolean(robot.nav?.navigating || snap?.navigating);
  const connected = robot.online && (health?.online ?? true);
  const linkLabel = connected
    ? "Online"
    : robot.online
      ? "Offline"
      : "연결실패";
  const assign = robot.assignment;
  const order = assign?.order;
  const itemsShort = order
    ? order.items.map((it) => `${it.productName}×${it.quantity}`).join(", ")
    : "";
  const battPct = snap?.battery?.percent;
  const usRange = snap?.ultrasonic?.rangeM;

  return (
    <section className="cart-status-panel" style={{ borderLeftColor: color }}>
      <div className="cart-status-compact-head">
        <span className="cart-color-swatch" style={{ background: color }} />
        <strong className="cart-status-name">{robot.label}</strong>
        <span className={`status-dot ${connected ? "on" : "off"}`} />
        <span className="cart-status-link">{linkLabel}</span>
        <span className="cart-status-pill">
          {navigating ? "주행" : "대기"}
        </span>
        {backend === "mock" ? (
          <span className="error cart-status-pill">mock</span>
        ) : null}
      </div>

      {robot.error ? (
        <p className="error cart-status-err">{robot.error}</p>
      ) : null}

      <div className="cart-kv">
        <div>
          <span>배터리</span>
          <b>
            {fmt(battPct, 0)}%
            {snap?.battery?.voltage != null
              ? ` · ${fmt(snap.battery.voltage, 2)}V`
              : ""}
            {battDummy ? <em className="error"> 더미</em> : null}
          </b>
        </div>
        <div className="cart-batt-mini">
          <i
            style={{
              width: `${Math.min(100, Math.max(0, battPct ?? 0))}%`,
              background: color,
            }}
          />
        </div>
        <div>
          <span>초음파</span>
          <b>
            {fmt(usRange, 2)}m
            {(snap?.ultrasonic?.irRaw || []).length
              ? ` · IR ${(snap?.ultrasonic?.irRaw || []).join("/")}`
              : ""}
            {usDummy ? <em className="error"> 더미</em> : null}
          </b>
        </div>
        <div>
          <span>좌표</span>
          <b className="cart-mono">
            {pose
              ? `${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}, ${pose.yaw.toFixed(2)}`
              : "—"}
          </b>
        </div>
        <div>
          <span>주문</span>
          <b>
            {order
              ? `#${order.id} · ${missionStatusLabel(assign?.status || "")}${
                  assign?.currentWaypoint
                    ? ` · ${assign.currentWaypoint}`
                    : ""
                }`
              : "없음 (idle)"}
          </b>
        </div>
        {order && assign?.currentWaypointLabel ? (
          <div>
            <span>작업</span>
            <b className="cart-aruco-phase">{assign.currentWaypointLabel}</b>
          </div>
        ) : null}
        {order ? (
          <div>
            <span>품목</span>
            <b className="cart-items-one-line" title={itemsShort}>
              {itemsShort || "—"}
              {` · ${order.totalPrice.toLocaleString("ko-KR")}원`}
            </b>
          </div>
        ) : null}
      </div>
    </section>
  );
}

export default function AdminRobotPage() {
  const [robots, setRobots] = useState<RobotMonitor[]>([]);
  const [devices, setDevices] = useState<Device[]>([]);
  const [missions, setMissions] = useState<MissionListItem[]>([]);
  const [queueLength, setQueueLength] = useState(0);
  const [error, setError] = useState<string | null>(null);
  const [updatedAt, setUpdatedAt] = useState<string | null>(null);
  const [auto, setAuto] = useState(true);
  const [controlRobotId, setControlRobotId] = useState<string>("");
  const [logMission, setLogMission] = useState<MissionDetail | null>(null);
  const [logLoading, setLogLoading] = useState(false);
  const [logError, setLogError] = useState<string | null>(null);
  const mapColRef = useRef<HTMLDivElement>(null);
  const sideColRef = useRef<HTMLDivElement>(null);

  const openMissionLogs = useCallback(async (missionId: number) => {
    setLogLoading(true);
    setLogError(null);
    setLogMission(null);
    try {
      const detail = await api<MissionDetail>(
        `/admin/robot/missions/${missionId}`,
      );
      setLogMission(detail);
    } catch (e) {
      setLogError(e instanceof Error ? e.message : "로그를 불러오지 못했습니다");
    } finally {
      setLogLoading(false);
    }
  }, []);

  const closeMissionLogs = useCallback(() => {
    setLogMission(null);
    setLogError(null);
    setLogLoading(false);
  }, []);

  useEffect(() => {
    if (!logLoading && !logMission && !logError) return;
    const onKey = (e: KeyboardEvent) => {
      if (e.key === "Escape") closeMissionLogs();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [logLoading, logMission, logError, closeMissionLogs]);

  const syncSideHeight = useCallback(() => {
    const left = mapColRef.current;
    const right = sideColRef.current;
    if (!left || !right) return;
    // 왼쪽(맵 600 고정) 높이에 오른쪽을 맞춤
    right.style.height = `${left.offsetHeight}px`;
  }, []);

  const refresh = useCallback(async () => {
    try {
      const [res, d, m] = await Promise.all([
        api<RobotsResponse>("/admin/robots"),
        api<Device[]>("/admin/robot/devices").catch(() => [] as Device[]),
        api<MissionListItem[]>("/admin/robot/missions").catch(
          () => [] as MissionListItem[],
        ),
      ]);
      const list = res.robots || [];
      setRobots(list);
      setQueueLength(res.queueLength ?? 0);
      setDevices(d);
      setMissions(Array.isArray(m) ? m : []);
      setError(null);
      setUpdatedAt(new Date().toLocaleTimeString("ko-KR"));
      setControlRobotId((prev) => {
        if (prev && list.some((r) => r.id === prev)) return prev;
        return list[0]?.id || "";
      });
    } catch (err) {
      setError(
        err instanceof Error
          ? err.message
          : "로봇 모니터링 API에 연결할 수 없습니다",
      );
    }
  }, []);

  useEffect(() => {
    void refresh();
  }, [refresh]);

  useEffect(() => {
    if (!auto) return;
    const id = window.setInterval(() => void refresh(), 2000);
    return () => window.clearInterval(id);
  }, [auto, refresh]);

  const mapRobotId = useMemo(() => {
    const online = robots.find((r) => r.online);
    return online?.id || robots[0]?.id || "";
  }, [robots]);

  const mapOverlays = useMemo(
    () =>
      robots.map((r, i) => ({
        id: r.id,
        label: r.label,
        color: robotColor(r.id, i),
        pose: r.nav?.pose || r.sensors?.pose || null,
        navigating: Boolean(r.nav?.navigating || r.sensors?.navigating),
        lidarPoints: r.sensors?.lidar?.points || [],
      })),
    [robots],
  );

  // 왼쪽 맵(뷰포트 600 고정) 높이에 오른쪽 패널을 맞춤
  useEffect(() => {
    const left = mapColRef.current;
    if (!left) return;
    const run = () => syncSideHeight();
    run();
    const ro =
      typeof ResizeObserver !== "undefined"
        ? new ResizeObserver(run)
        : null;
    ro?.observe(left);
    window.addEventListener("resize", run);
    return () => {
      ro?.disconnect();
      window.removeEventListener("resize", run);
    };
  }, [syncSideHeight, robots.length, missions.length, mapRobotId, controlRobotId]);

  return (
    <div className="admin-panel">
      <div className="admin-panel-head">
        <div>
          <h1 className="hero-title" style={{ fontSize: "2.2rem" }}>
            로봇 모니터링
          </h1>
          <p className="muted">
            공유 맵 · 카트별 상태
            {robots.length ? ` (${robots.length}대)` : ""}
            {queueLength > 0 ? ` · 대기 큐 ${queueLength}건` : ""}
          </p>
        </div>
        <div className="admin-panel-actions">
          <label className="admin-check">
            <input
              type="checkbox"
              checked={auto}
              onChange={(e) => setAuto(e.target.checked)}
            />
            자동 갱신
          </label>
          <button
            type="button"
            className="btn secondary"
            onClick={() => void refresh()}
          >
            새로고침
          </button>
        </div>
      </div>

      {error ? (
        <p className="error">
          {error}
          <span className="muted">
            {" "}
            — BFF의 `PINKY_ROBOTS` 또는 `PINKY_URL` / `PINKY_URL_2`를 확인하세요.
          </span>
        </p>
      ) : null}
      {updatedAt ? (
        <p className="muted" style={{ marginTop: 0 }}>
          마지막 갱신 {updatedAt}
        </p>
      ) : null}

      {!robots.length && !error ? (
        <p className="muted">등록된 로봇이 없습니다.</p>
      ) : null}

      {robots.length ? (
        <div className="monitor-split">
          <div className="monitor-map-col" ref={mapColRef}>
            <div className="monitor-card monitor-map-card">
              <h3>맵 · 네비게이션</h3>
              <p className="muted" style={{ marginTop: 0, fontSize: "0.85rem" }}>
                두 카트 pose를 한 맵에 표시합니다. 드롭다운으로 조종할 로봇을 고른 뒤
                좌드래그(pose) · 우드래그(goal)로 제어하세요.
              </p>
              <div className="map-robot-legend">
                {robots.map((r, i) => (
                  <span key={r.id} className="map-robot-legend-item">
                    <i style={{ background: robotColor(r.id, i) }} />
                    {r.label}
                  </span>
                ))}
              </div>
              {mapRobotId && controlRobotId ? (
                <OccupancyNavMap
                  mapRobotId={mapRobotId}
                  robots={mapOverlays}
                  controlRobotId={controlRobotId}
                  onControlRobotChange={setControlRobotId}
                />
              ) : null}
            </div>
          </div>

          <div className="monitor-side-col" ref={sideColRef}>
            {robots.map((r, i) => (
              <CartStatusPanel
                key={r.id}
                robot={r}
                color={robotColor(r.id, i)}
              />
            ))}
            <section className="order-list-panel">
              <header className="order-list-head">
                <h3>주문 목록</h3>
                <span className="muted">{missions.length}건</span>
              </header>
              <div className="order-list-scroll">
                {missions.length ? (
                  <ul className="order-list">
                    {missions.map((m) => {
                      const order = m.order;
                      const items = order?.items || [];
                      const itemsShort = items
                        .map((it) => `${it.productName}×${it.quantity}`)
                        .join(", ");
                      const assigned = Boolean(m.deviceCode);
                      const cartColor = m.deviceCode
                        ? robotColor(m.deviceCode, 0)
                        : undefined;
                      return (
                        <li key={m.id}>
                          <button
                            type="button"
                            className="order-list-item order-list-item-btn"
                            onClick={() => void openMissionLogs(m.id)}
                            title="작업 로그 보기"
                          >
                          <div className="order-list-row">
                            <strong>#{m.orderId}</strong>
                            <span className="order-list-status">
                              {missionStatusLabel(m.status)}
                            </span>
                            {assigned ? (
                              <span
                                className="order-list-cart"
                                style={
                                  cartColor
                                    ? {
                                        borderColor: cartColor,
                                        color: cartColor,
                                      }
                                    : undefined
                                }
                              >
                                {m.deviceCode}
                              </span>
                            ) : (
                              <span className="order-list-cart muted">
                                미할당
                              </span>
                            )}
                          </div>
                          <div className="order-list-meta">
                            <span>
                              {(order?.totalPrice ?? 0).toLocaleString("ko-KR")}
                              원
                            </span>
                            {m.currentWaypoint ? (
                              <span>· {m.currentWaypoint}</span>
                            ) : null}
                            {m.currentWaypointLabel ? (
                              <span className="order-list-phase">
                                · {m.currentWaypointLabel}
                              </span>
                            ) : null}
                            {m.createdAt || order?.createdAt ? (
                              <span className="muted">
                                ·{" "}
                                {new Date(
                                  m.createdAt || order?.createdAt || "",
                                ).toLocaleString("ko-KR", {
                                  month: "numeric",
                                  day: "numeric",
                                  hour: "2-digit",
                                  minute: "2-digit",
                                })}
                              </span>
                            ) : null}
                          </div>
                          {itemsShort ? (
                            <p className="order-list-items" title={itemsShort}>
                              {itemsShort}
                            </p>
                          ) : null}
                          </button>
                        </li>
                      );
                    })}
                  </ul>
                ) : (
                  <p className="muted" style={{ margin: 0, fontSize: "0.85rem" }}>
                    주문이 없습니다.
                  </p>
                )}
              </div>
            </section>
          </div>
        </div>
      ) : null}

      <section className="robot-block robot-block-devices">
        <header className="robot-block-head">
          <h2 className="robot-block-title">디바이스 (Controller)</h2>
        </header>
        <table className="table">
          <thead>
            <tr>
              <th>코드</th>
              <th>타입</th>
              <th>상태</th>
            </tr>
          </thead>
          <tbody>
            {devices.map((d) => (
              <tr key={d.id}>
                <td>{d.code}</td>
                <td>{d.type}</td>
                <td>{d.status}</td>
              </tr>
            ))}
            {!devices.length ? (
              <tr>
                <td colSpan={3} className="muted">
                  디바이스 없음
                </td>
              </tr>
            ) : null}
          </tbody>
        </table>
      </section>

      {logLoading || logMission || logError ? (
        <div
          className="mission-log-backdrop"
          role="presentation"
          onClick={closeMissionLogs}
        >
          <div
            className="mission-log-modal"
            role="dialog"
            aria-modal="true"
            aria-labelledby="mission-log-title"
            onClick={(e) => e.stopPropagation()}
          >
            <header className="mission-log-head">
              <div>
                <h3 id="mission-log-title">
                  {logMission
                    ? `주문 #${logMission.orderId} 작업 로그`
                    : "작업 로그"}
                </h3>
                {logMission ? (
                  <p className="mission-log-sub muted">
                    미션 #{logMission.id} ·{" "}
                    {missionStatusLabel(logMission.status)}
                    {logMission.deviceCode
                      ? ` · ${logMission.deviceCode}`
                      : ""}
                    {logMission.currentWaypointLabel ||
                    logMission.currentWaypoint
                      ? ` · ${
                          logMission.currentWaypointLabel ||
                          logMission.currentWaypoint
                        }`
                      : ""}
                  </p>
                ) : null}
              </div>
              <button
                type="button"
                className="mission-log-close"
                onClick={closeMissionLogs}
              >
                닫기
              </button>
            </header>

            {logLoading ? (
              <p className="muted mission-log-empty">불러오는 중…</p>
            ) : null}
            {logError ? <p className="error mission-log-empty">{logError}</p> : null}

            {!logLoading && logMission ? (
              <>
                {logMission.order?.items?.length ? (
                  <p className="mission-log-items">
                    {logMission.order.items
                      .map((it) => `${it.productName}×${it.quantity}`)
                      .join(", ")}
                    {` · ${(logMission.order.totalPrice ?? 0).toLocaleString("ko-KR")}원`}
                  </p>
                ) : null}
                <ol className="mission-log-list">
                  {(logMission.events || []).length ? (
                    (logMission.events || []).map((ev) => {
                      const fromL = ev.fromStatus
                        ? missionStatusLabel(ev.fromStatus)
                        : "—";
                      const toL = ev.toStatus
                        ? missionStatusLabel(ev.toStatus)
                        : "—";
                      const same = ev.fromStatus === ev.toStatus;
                      return (
                        <li key={ev.id} className="mission-log-entry">
                          <time className="mission-log-time">
                            {ev.createdAt
                              ? new Date(ev.createdAt).toLocaleString("ko-KR", {
                                  month: "numeric",
                                  day: "numeric",
                                  hour: "2-digit",
                                  minute: "2-digit",
                                  second: "2-digit",
                                })
                              : "—"}
                          </time>
                          <div className="mission-log-body">
                            <span className="mission-log-status">
                              {same ? toL : `${fromL} → ${toL}`}
                            </span>
                            {ev.note ? (
                              <span className="mission-log-note">{ev.note}</span>
                            ) : null}
                          </div>
                        </li>
                      );
                    })
                  ) : (
                    <li className="muted mission-log-empty">
                      기록된 이벤트가 없습니다.
                    </li>
                  )}
                </ol>
              </>
            ) : null}
          </div>
        </div>
      ) : null}
    </div>
  );
}
