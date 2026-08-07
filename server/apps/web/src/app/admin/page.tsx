"use client";

import { useCallback, useEffect, useState } from "react";
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
};

type Lidar = {
  rangesCount?: number;
  rangesSample?: number[];
  range_min?: number;
  range_max?: number;
  rangeMin?: number;
  rangeMax?: number;
  frame_id?: string;
  frameId?: string;
  stamp?: number | null;
  points?: { x: number; y: number; r?: number }[];
  angleMin?: number;
  angleIncrement?: number;
};

type Imu = {
  orientation?: { x: number; y: number; z: number; w: number };
  angularVelocity?: { x: number; y: number; z: number };
  linearAcceleration?: { x: number; y: number; z: number };
  frameId?: string;
  stamp?: number | null;
};

type Ultrasonic = {
  rangeM?: number | null;
  minRange?: number;
  maxRange?: number;
  irRaw?: number[];
  frameId?: string;
};

type Snapshot = {
  deviceCode?: string;
  backend?: string;
  online?: boolean;
  battery?: Battery;
  lidar?: Lidar;
  imu?: Imu;
  ultrasonic?: Ultrasonic;
  pose?: { x: number; y: number; yaw: number } | null;
  navigating?: boolean;
  hasData?: {
    battery?: boolean;
    lidar?: boolean;
    imu?: boolean;
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

function fmt(n: number | null | undefined, digits = 2): string {
  if (n == null || Number.isNaN(n)) return "—";
  return n.toFixed(digits);
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

function RobotBlock({ robot }: { robot: RobotMonitor }) {
  const health = robot.health;
  const snap = robot.sensors;
  const hd = snap?.hasData;
  const allMissing =
    hd && !hd.battery && !hd.imu && !hd.ultrasonic && !hd.lidar;
  const partial =
    hd &&
    (hd.battery || hd.imu || hd.ultrasonic || hd.lidar) &&
    (snap?.warnings?.length ?? 0) > 0;

  return (
    <section className="robot-block">
      <header className="robot-block-head">
        <div>
          <h2 className="robot-block-title">{robot.label}</h2>
          <p className="muted robot-block-url">{robot.url}</p>
        </div>
        <div className="robot-block-status">
          <span
            className={`status-dot ${robot.online && (health?.online ?? true) ? "on" : "off"}`}
          />
          {robot.online
            ? health?.online === false
              ? "Offline"
              : "Online"
            : "연결 실패"}
        </div>
      </header>

      {robot.error ? (
        <p className="error" style={{ marginBottom: "0.75rem" }}>
          {robot.error}
        </p>
      ) : null}

      {(health?.backend || snap?.backend) === "mock" ? (
        <div className="error" style={{ marginBottom: "0.75rem" }}>
          <strong>더미(mock) 백엔드입니다</strong>
          <p className="muted" style={{ margin: "0.5rem 0 0" }}>
            BFF는 로봇(<code>{robot.url}</code>)에 정상 연결되었지만, 로봇의{" "}
            <code>run.py</code>가 <code>PINKY_BACKEND=mock</code> 으로 떠 있습니다.
            라즈베리에서 <code>pinky.env</code>에 <code>PINKY_BACKEND=ros2</code> 확인 후
            기존 프로세스를 종료하고 재시작하세요.
          </p>
          <pre
            className="muted"
            style={{
              margin: "0.6rem 0 0",
              whiteSpace: "pre-wrap",
              fontSize: "0.85rem",
            }}
          >
            {`# 로봇(SSH)에서
curl -sS http://127.0.0.1:4200/health   # backend 확인
pkill -f 'python.*run.py' || true
cd ~/pinky && source /opt/ros/jazzy/setup.bash
.venv/bin/python run.py
# 로그에 backend=ros2 가 보여야 함`}
          </pre>
        </div>
      ) : null}

      {allMissing ? (
        <div className="error" style={{ marginBottom: "0.75rem" }}>
          <strong>센서 데이터가 없습니다</strong>
          <p className="muted" style={{ margin: "0.5rem 0 0" }}>
            로봇 `run.py`(PINKY_BACKEND=ros2)와 해당 URL을 확인하세요.
          </p>
        </div>
      ) : null}

      {partial ? (
        <div
          className="muted"
          style={{
            marginBottom: "0.75rem",
            padding: "0.75rem 1rem",
            border: "1px solid var(--line)",
            background: "rgba(255,255,255,0.4)",
          }}
        >
          <strong style={{ color: "var(--ink)" }}>일부 센서만 수신</strong>
          <ul style={{ margin: "0.4rem 0 0", paddingLeft: "1.2rem" }}>
            {(snap?.warnings || []).map((w) => (
              <li key={w}>{w}</li>
            ))}
          </ul>
        </div>
      ) : null}

      <div className="monitor-grid">
        <div className="monitor-card monitor-card-wide">
          <h3>할당 주문</h3>
          {robot.assignment?.order ? (
            <>
              <dl className="monitor-dl">
                <div>
                  <dt>주문</dt>
                  <dd>#{robot.assignment.order.id}</dd>
                </div>
                <div>
                  <dt>상태</dt>
                  <dd>{missionStatusLabel(robot.assignment.status)}</dd>
                </div>
                <div>
                  <dt>경유지</dt>
                  <dd>
                    {robot.assignment.currentWaypoint
                      ? `${robot.assignment.currentWaypoint}${
                          robot.assignment.currentWaypointLabel
                            ? ` · ${robot.assignment.currentWaypointLabel}`
                            : ""
                        }`
                      : "—"}
                  </dd>
                </div>
                <div>
                  <dt>합계</dt>
                  <dd>
                    {robot.assignment.order.totalPrice.toLocaleString("ko-KR")}원
                  </dd>
                </div>
              </dl>
              <ul
                style={{
                  margin: "0.5rem 0 0",
                  paddingLeft: "1.2rem",
                  fontSize: "0.9rem",
                }}
              >
                {robot.assignment.order.items.map((it, idx) => (
                  <li key={`${it.productName}-${idx}`}>
                    {it.productName} × {it.quantity}
                  </li>
                ))}
              </ul>
            </>
          ) : (
            <p className="muted" style={{ margin: 0 }}>
              대기 중 (idle) — 할당된 주문이 없습니다.
            </p>
          )}
        </div>

        <div className="monitor-card">
          <h3>연결</h3>
          <dl className="monitor-dl">
            <div>
              <dt>상태</dt>
              <dd>
                <span
                  className={`status-dot ${robot.online && (health?.online ?? true) ? "on" : "off"}`}
                />
                {robot.online
                  ? health?.online === false
                    ? "Offline"
                    : "Online"
                  : "연결 실패"}
              </dd>
            </div>
            <div>
              <dt>Backend</dt>
              <dd>{health?.backend || snap?.backend || "—"}</dd>
            </div>
            <div>
              <dt>Device</dt>
              <dd>{health?.deviceCode || snap?.deviceCode || robot.id}</dd>
            </div>
            <div>
              <dt>Publisher</dt>
              <dd>
                {health?.sensorPublisher == null
                  ? "—"
                  : health.sensorPublisher
                    ? "ON"
                    : "OFF"}
              </dd>
            </div>
          </dl>
        </div>

        <div className="monitor-card">
          <h3>배터리</h3>
          <p className="monitor-metric">
            {fmt(snap?.battery?.percent, 1)}
            <small>%</small>
          </p>
          <dl className="monitor-dl">
            <div>
              <dt>전압</dt>
              <dd>{fmt(snap?.battery?.voltage, 3)} V</dd>
            </div>
            <div>
              <dt>소스</dt>
              <dd>
                {snap?.battery?.source || "—"}
                {(snap?.battery?.source === "mock" ||
                  (health?.backend || snap?.backend) === "mock") && (
                  <span className="error"> (더미)</span>
                )}
              </dd>
            </div>
          </dl>
          <div className="battery-bar">
            <i
              style={{
                width: `${Math.min(100, Math.max(0, snap?.battery?.percent ?? 0))}%`,
              }}
            />
          </div>
          {!snap?.hasData?.battery ? (
            <p className="muted" style={{ marginTop: "0.75rem", marginBottom: 0 }}>
              측정값 없음
            </p>
          ) : null}
        </div>

        <div className="monitor-card">
          <h3>초음파 / IR</h3>
          <p className="monitor-metric">
            {fmt(snap?.ultrasonic?.rangeM, 3)}
            <small>m</small>
          </p>
          <dl className="monitor-dl">
            <div>
              <dt>범위</dt>
              <dd>
                {fmt(snap?.ultrasonic?.minRange, 2)} –{" "}
                {fmt(snap?.ultrasonic?.maxRange, 2)} m
              </dd>
            </div>
            <div>
              <dt>IR raw</dt>
              <dd>{(snap?.ultrasonic?.irRaw || []).join(", ") || "—"}</dd>
            </div>
          </dl>
        </div>

        <div className="monitor-card monitor-card-wide">
          <h3>IMU</h3>
          <div className="imu-grid">
            <div>
              <h4>Orientation</h4>
              <pre>
                {JSON.stringify(snap?.imu?.orientation || {}, null, 2)}
              </pre>
            </div>
            <div>
              <h4>Angular velocity</h4>
              <pre>
                {JSON.stringify(snap?.imu?.angularVelocity || {}, null, 2)}
              </pre>
            </div>
            <div>
              <h4>Linear acceleration</h4>
              <pre>
                {JSON.stringify(snap?.imu?.linearAcceleration || {}, null, 2)}
              </pre>
            </div>
          </div>
        </div>

        <div className="monitor-card monitor-card-wide">
          <h3>맵 · 네비게이션</h3>
          <p className="muted" style={{ marginTop: 0, fontSize: "0.85rem" }}>
            좌드래그: 현재 pose · 우드래그: 목표 위치+최종 yaw (Nav2 goal)
          </p>
          <dl className="monitor-dl">
            <div>
              <dt>라이다</dt>
              <dd>
                {snap?.lidar?.points?.length ?? 0} pts
                {robot.nav?.mapId ? ` · map ${robot.nav.mapId}` : ""}
              </dd>
            </div>
            <div>
              <dt>주행</dt>
              <dd>
                {robot.nav?.navigating || snap?.navigating
                  ? "navigating"
                  : "idle"}
              </dd>
            </div>
            <div>
              <dt>현재 좌표</dt>
              <dd>
                {(() => {
                  const p = robot.nav?.pose || snap?.pose;
                  return p
                    ? `x=${p.x.toFixed(3)}, y=${p.y.toFixed(3)}, yaw=${p.yaw.toFixed(3)}`
                    : "—";
                })()}
              </dd>
            </div>
          </dl>
          <OccupancyNavMap
            robotId={robot.id}
            lidarPoints={snap?.lidar?.points || []}
            pose={
              robot.nav?.pose ||
              snap?.pose ||
              null
            }
            navigating={Boolean(robot.nav?.navigating || snap?.navigating)}
          />
        </div>
      </div>
    </section>
  );
}

export default function AdminRobotPage() {
  const [robots, setRobots] = useState<RobotMonitor[]>([]);
  const [devices, setDevices] = useState<Device[]>([]);
  const [queueLength, setQueueLength] = useState(0);
  const [error, setError] = useState<string | null>(null);
  const [updatedAt, setUpdatedAt] = useState<string | null>(null);
  const [auto, setAuto] = useState(true);

  const refresh = useCallback(async () => {
    try {
      const [res, d] = await Promise.all([
        api<RobotsResponse>("/admin/robots"),
        api<Device[]>("/admin/robot/devices").catch(() => [] as Device[]),
      ]);
      setRobots(res.robots || []);
      setQueueLength(res.queueLength ?? 0);
      setDevices(d);
      setError(null);
      setUpdatedAt(new Date().toLocaleTimeString("ko-KR"));
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

  return (
    <div className="admin-panel">
      <div className="admin-panel-head">
        <div>
          <h1 className="hero-title" style={{ fontSize: "2.2rem" }}>
            로봇 모니터링
          </h1>
          <p className="muted">
            주행로봇별 영역 — 연결 · 할당 주문 · 배터리 · Occupancy 맵/네비
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

      <div className="robot-stack">
        {robots.map((r) => (
          <RobotBlock key={r.id} robot={r} />
        ))}
        {!robots.length && !error ? (
          <p className="muted">등록된 로봇이 없습니다.</p>
        ) : null}
      </div>

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
    </div>
  );
}
