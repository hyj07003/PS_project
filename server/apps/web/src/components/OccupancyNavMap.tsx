"use client";

import { useCallback, useEffect, useRef, useState } from "react";
import { api, getApiUrl, getToken } from "@/lib/api";

export type MapMeta = {
  mapId?: string;
  resolution: number;
  origin: { x: number; y: number; yaw?: number };
  width: number;
  height: number;
};

export type NavPose = { x: number; y: number; yaw: number };

export type LidarPoint = { x: number; y: number; r?: number };

type DragMode = "pose" | "goal";

type Props = {
  robotId: string;
  lidarPoints?: LidarPoint[];
  pose?: NavPose | null;
  navigating?: boolean;
  className?: string;
  onPoseSet?: (pose: NavPose) => void;
  onGoalSet?: (goal: NavPose) => void;
};

function worldToPixel(
  meta: MapMeta,
  x: number,
  y: number,
): { col: number; row: number } {
  const col = (x - meta.origin.x) / meta.resolution;
  const row = meta.height - 1 - (y - meta.origin.y) / meta.resolution;
  return { col, row };
}

function pixelToWorld(
  meta: MapMeta,
  col: number,
  row: number,
): { x: number; y: number } {
  const x = meta.origin.x + col * meta.resolution;
  const y = meta.origin.y + (meta.height - 1 - row) * meta.resolution;
  return { x, y };
}

/**
 * Occupancy 맵 + 라이다.
 * 좌클릭 드래그: initialpose (위치+yaw)
 * 우클릭 드래그: Nav2 goal (위치+최종 yaw)
 */
export function OccupancyNavMap({
  robotId,
  lidarPoints = [],
  pose = null,
  navigating = false,
  className,
  onPoseSet,
  onGoalSet,
}: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const imgRef = useRef<HTMLImageElement | null>(null);
  const [meta, setMeta] = useState<MapMeta | null>(null);
  const [mapUrl, setMapUrl] = useState<string | null>(null);
  const [status, setStatus] = useState<string | null>(null);
  const [lastGoal, setLastGoal] = useState<NavPose | null>(null);
  const [busy, setBusy] = useState(false);
  const dragRef = useRef<{
    active: boolean;
    mode: DragMode;
    startCol: number;
    startRow: number;
    curCol: number;
    curRow: number;
  } | null>(null);
  const [, setTick] = useState(0);

  const q = robotId ? `?robot=${encodeURIComponent(robotId)}` : "";

  useEffect(() => {
    let revoked: string | null = null;
    let cancelled = false;
    (async () => {
      try {
        const m = await api<MapMeta>(`/admin/robot/map/meta${q}`);
        if (cancelled) return;
        setMeta(m);
        const token = getToken();
        const res = await fetch(`${getApiUrl()}/admin/robot/map/image${q}`, {
          headers: token ? { Authorization: `Bearer ${token}` } : {},
        });
        if (!res.ok) throw new Error(`map image ${res.status}`);
        const blob = await res.blob();
        const url = URL.createObjectURL(blob);
        revoked = url;
        const img = new Image();
        img.onload = () => {
          if (cancelled) return;
          imgRef.current = img;
          setMapUrl(url);
        };
        img.src = url;
      } catch (err) {
        if (!cancelled) {
          setStatus(
            err instanceof Error ? err.message : "맵을 불러오지 못했습니다",
          );
        }
      }
    })();
    return () => {
      cancelled = true;
      if (revoked) URL.revokeObjectURL(revoked);
    };
  }, [q]);

  const redraw = useCallback(() => {
    const canvas = canvasRef.current;
    const img = imgRef.current;
    if (!canvas || !meta) return;
    const parent = canvas.parentElement;
    const maxW = Math.min(parent?.clientWidth || 640, 720);
    const scale = maxW / meta.width;
    const w = Math.round(meta.width * scale);
    const h = Math.round(meta.height * scale);
    const dpr = window.devicePixelRatio || 1;
    canvas.width = w * dpr;
    canvas.height = h * dpr;
    canvas.style.width = `${w}px`;
    canvas.style.height = `${h}px`;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    ctx.fillStyle = "#e8e6e1";
    ctx.fillRect(0, 0, w, h);
    if (img) {
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(img, 0, 0, w, h);
    }

    const toScreen = (col: number, row: number) => ({
      sx: col * scale,
      sy: row * scale,
    });

    // LaserScan → map: 앞뒤+좌우 반전 ≡ 180° (원래 표시)
    if (pose && lidarPoints.length) {
      const c = Math.cos(pose.yaw);
      const s = Math.sin(pose.yaw);
      ctx.fillStyle = "rgba(80, 120, 90, 0.85)";
      for (const p of lidarPoints) {
        if (!Number.isFinite(p.x) || !Number.isFinite(p.y)) continue;
        const lx = -p.x;
        const ly = -p.y;
        const mx = pose.x + c * lx - s * ly;
        const my = pose.y + s * lx + c * ly;
        const { col, row } = worldToPixel(meta, mx, my);
        const { sx, sy } = toScreen(col, row);
        ctx.beginPath();
        ctx.arc(sx, sy, 1.6, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    if (pose) {
      const { col, row } = worldToPixel(meta, pose.x, pose.y);
      const { sx, sy } = toScreen(col, row);
      const len = 14;
      ctx.save();
      ctx.translate(sx, sy);
      ctx.rotate(-pose.yaw - Math.PI / 2);
      ctx.fillStyle = navigating ? "#c45c26" : "#2c2c2c";
      ctx.beginPath();
      ctx.moveTo(0, -len);
      ctx.lineTo(-7, len * 0.55);
      ctx.lineTo(7, len * 0.55);
      ctx.closePath();
      ctx.fill();
      ctx.restore();
    }

    const drag = dragRef.current;
    if (drag?.active) {
      const a = toScreen(drag.startCol, drag.startRow);
      const b = toScreen(drag.curCol, drag.curRow);
      const isGoal = drag.mode === "goal";
      ctx.strokeStyle = isGoal
        ? "rgba(196, 92, 38, 0.95)"
        : "rgba(80, 100, 180, 0.9)";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(a.sx, a.sy);
      ctx.lineTo(b.sx, b.sy);
      ctx.stroke();
      ctx.fillStyle = ctx.strokeStyle;
      ctx.beginPath();
      ctx.arc(a.sx, a.sy, 4, 0, Math.PI * 2);
      ctx.fill();
      // 화살표 머리 (드래그 방향 = 최종 yaw)
      const ang = Math.atan2(b.sy - a.sy, b.sx - a.sx);
      ctx.beginPath();
      ctx.moveTo(b.sx, b.sy);
      ctx.lineTo(
        b.sx - 10 * Math.cos(ang - 0.4),
        b.sy - 10 * Math.sin(ang - 0.4),
      );
      ctx.lineTo(
        b.sx - 10 * Math.cos(ang + 0.4),
        b.sy - 10 * Math.sin(ang + 0.4),
      );
      ctx.closePath();
      ctx.fill();
    }

    ctx.fillStyle = "rgba(60,55,50,0.85)";
    ctx.font = "12px sans-serif";
    ctx.fillText(
      `${meta.mapId || "map"} · ${meta.width}×${meta.height} · ${meta.resolution}m`,
      8,
      h - 10,
    );
    ctx.fillText("좌드래그: 초기 pose · 우드래그: 목표+yaw", 8, 16);
  }, [meta, lidarPoints, pose, navigating]);

  useEffect(() => {
    redraw();
  }, [redraw, mapUrl]);

  const eventToPixel = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    if (!canvas || !meta) return null;
    const rect = canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;
    const scale = rect.width / meta.width;
    return { col: sx / scale, row: sy / scale };
  };

  const finishDrag = async (
    mode: DragMode,
    startCol: number,
    startRow: number,
    curCol: number,
    curRow: number,
  ) => {
    if (!meta || busy) return;
    const start = pixelToWorld(meta, startCol, startRow);
    const end = pixelToWorld(meta, curCol, curRow);
    const dx = end.x - start.x;
    const dy = end.y - start.y;
    // 드래그가 거의 없으면 yaw=0 (또는 현재 pose yaw 유지하지 않고 0)
    const yaw =
      Math.hypot(dx, dy) < meta.resolution * 0.5
        ? 0
        : Math.atan2(dy, dx);
    const body = { x: start.x, y: start.y, yaw };
    setBusy(true);
    setStatus(null);
    try {
      if (mode === "pose") {
        const res = await api<{ success?: boolean; message?: string }>(
          `/admin/robot/nav/initialpose${q}`,
          { method: "POST", body: JSON.stringify(body) },
        );
        if (!res.success) {
          setStatus(res.message || "initialpose 실패");
        } else {
          setStatus(
            `pose (${start.x.toFixed(2)}, ${start.y.toFixed(2)}, yaw=${yaw.toFixed(2)})`,
          );
          onPoseSet?.(body);
        }
      } else {
        const res = await api<{ success?: boolean; message?: string }>(
          `/admin/robot/nav/goal${q}`,
          { method: "POST", body: JSON.stringify(body) },
        );
        if (!res.success) {
          setStatus(res.message || "Nav2 액션 서버 없음 / goal 실패");
        } else {
          setLastGoal(body);
          setStatus(
            `goal (${start.x.toFixed(2)}, ${start.y.toFixed(2)}, yaw=${yaw.toFixed(2)})`,
          );
          onGoalSet?.(body);
        }
      }
    } catch (err) {
      setStatus(
        err instanceof Error
          ? err.message
          : mode === "pose"
            ? "initialpose 오류"
            : "goal 오류",
      );
    } finally {
      setBusy(false);
      setTick((n) => n + 1);
    }
  };

  const onMouseDown = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!meta || busy) return;
    if (e.button !== 0 && e.button !== 2) return;
    e.preventDefault();
    const p = eventToPixel(e);
    if (!p) return;
    dragRef.current = {
      active: true,
      mode: e.button === 2 ? "goal" : "pose",
      startCol: p.col,
      startRow: p.row,
      curCol: p.col,
      curRow: p.row,
    };
    setTick((n) => n + 1);
  };

  const onMouseMove = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!dragRef.current?.active) return;
    const p = eventToPixel(e);
    if (!p) return;
    dragRef.current.curCol = p.col;
    dragRef.current.curRow = p.row;
    setTick((n) => n + 1);
  };

  const onMouseUp = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!dragRef.current?.active) return;
    const drag = dragRef.current;
    // 시작한 버튼과 맞는 up 만 처리 (좌=0, 우=2)
    const expected = drag.mode === "goal" ? 2 : 0;
    if (e.button !== expected) return;
    dragRef.current = null;
    void finishDrag(
      drag.mode,
      drag.startCol,
      drag.startRow,
      drag.curCol,
      drag.curRow,
    );
  };

  const onStop = async () => {
    setBusy(true);
    try {
      await api(`/admin/robot/nav/stop${q}`, {
        method: "POST",
        body: "{}",
      });
      setStatus("정지 요청");
    } catch (err) {
      setStatus(err instanceof Error ? err.message : "stop 오류");
    } finally {
      setBusy(false);
    }
  };

  return (
    <div className={className || "occupancy-nav-map"}>
      <div className="occupancy-nav-toolbar">
        <button
          type="button"
          className="btn secondary"
          disabled={busy}
          onClick={() => void onStop()}
        >
          주행 정지
        </button>
        <span className="muted" style={{ fontSize: "0.85rem" }}>
          {navigating ? "주행 중" : "대기"}
        </span>
      </div>
      <dl className="monitor-dl occupancy-nav-coords">
        <div>
          <dt>현재</dt>
          <dd>
            {pose
              ? `x=${pose.x.toFixed(3)}  y=${pose.y.toFixed(3)}  yaw=${pose.yaw.toFixed(3)} rad (${((pose.yaw * 180) / Math.PI).toFixed(1)}°)`
              : "— (좌드래그로 pose 설정 또는 TF 대기)"}
          </dd>
        </div>
        <div>
          <dt>목표</dt>
          <dd>
            {lastGoal
              ? `x=${lastGoal.x.toFixed(3)}  y=${lastGoal.y.toFixed(3)}  yaw=${lastGoal.yaw.toFixed(3)} rad (${((lastGoal.yaw * 180) / Math.PI).toFixed(1)}°)`
              : "— (우드래그로 goal 지정)"}
          </dd>
        </div>
      </dl>
      <canvas
        ref={canvasRef}
        aria-label="Occupancy 네비 맵"
        onMouseDown={onMouseDown}
        onMouseMove={onMouseMove}
        onMouseUp={onMouseUp}
        onMouseLeave={() => {
          if (dragRef.current) {
            dragRef.current = null;
            setTick((n) => n + 1);
          }
        }}
        onContextMenu={(e) => e.preventDefault()}
        style={{ cursor: "crosshair", maxWidth: "100%", touchAction: "none" }}
      />
      {status ? (
        <p className="muted" style={{ margin: "0.5rem 0 0", fontSize: "0.85rem" }}>
          {status}
        </p>
      ) : null}
    </div>
  );
}
