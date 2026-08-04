"use client";

import { useCallback, useEffect, useState } from "react";
import { api } from "@/lib/api";
import { LidarMap } from "@/components/LidarMap";

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
  hasData?: {
    battery?: boolean;
    lidar?: boolean;
    imu?: boolean;
    ultrasonic?: boolean;
  };
  warnings?: string[];
};

type Device = {
  id: number;
  code: string;
  type: string;
  status: string;
};

type RobotMonitor = {
  id: string;
  label: string;
  url: string;
  online: boolean;
  health: RobotHealth | null;
  sensors: Snapshot | null;
  error: string | null;
};

type RobotsResponse = {
  robots: RobotMonitor[];
  count: number;
};

function fmt(n: number | null | undefined, digits = 2): string {
  if (n == null || Number.isNaN(n)) return "—";
  return n.toFixed(digits);
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
              <dd>{snap?.battery?.source || "—"}</dd>
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
          <h3>라이다 맵</h3>
          <dl className="monitor-dl">
            <div>
              <dt>포인트</dt>
              <dd>
                {snap?.lidar?.points?.length ?? 0} /{" "}
                {snap?.lidar?.rangesCount ?? "—"}
              </dd>
            </div>
            <div>
              <dt>거리 범위</dt>
              <dd>
                {fmt(snap?.lidar?.rangeMin ?? snap?.lidar?.range_min, 2)} –{" "}
                {fmt(snap?.lidar?.rangeMax ?? snap?.lidar?.range_max, 2)} m
              </dd>
            </div>
            <div>
              <dt>Frame</dt>
              <dd>
                {snap?.lidar?.frameId || snap?.lidar?.frame_id || "—"}
              </dd>
            </div>
          </dl>
          {snap?.lidar?.points?.length ? (
            <LidarMap
              points={snap.lidar.points}
              rangeMax={snap.lidar.rangeMax ?? snap.lidar.range_max ?? 8}
            />
          ) : (
            <p className="muted">
              라이다 포인트 없음 — `/dev/ttyAMA0` RPLidar 또는 sllidar를 확인하세요.
            </p>
          )}
        </div>
      </div>
    </section>
  );
}

export default function AdminRobotPage() {
  const [robots, setRobots] = useState<RobotMonitor[]>([]);
  const [devices, setDevices] = useState<Device[]>([]);
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
            주행로봇별 영역 — 연결 · 배터리 · 초음파 · IMU · 라이다맵
            {robots.length ? ` (${robots.length}대)` : ""}
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
