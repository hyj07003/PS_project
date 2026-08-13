"use client";

import { useCallback, useEffect, useMemo, useRef, useState } from "react";
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

export type MapRobotOverlay = {
  id: string;
  label: string;
  color: string;
  pose?: NavPose | null;
  navigating?: boolean;
  lidarPoints?: LidarPoint[];
};

type DragMode = "pose" | "goal";

type Props = {
  /** 맵 이미지/메타를 가져올 로봇 (공유 맵 — 보통 첫 온라인 로봇) */
  mapRobotId: string;
  robots: MapRobotOverlay[];
  /** pose/goal/stop 명령을 보낼 로봇 */
  controlRobotId: string;
  onControlRobotChange?: (id: string) => void;
  className?: string;
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

function drawRobotMarker(
  ctx: CanvasRenderingContext2D,
  sx: number,
  sy: number,
  yaw: number,
  color: string,
  navigating: boolean,
  label: string,
) {
  const len = 8;
  ctx.save();
  ctx.translate(sx, sy);
  ctx.rotate(-yaw - Math.PI / 2);
  ctx.fillStyle = color;
  ctx.strokeStyle = navigating ? "#1a1a1a" : "rgba(0,0,0,0.35)";
  ctx.lineWidth = navigating ? 1.5 : 0.8;
  ctx.beginPath();
  ctx.moveTo(0, -len);
  ctx.lineTo(-4, len * 0.55);
  ctx.lineTo(4, len * 0.55);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();

  ctx.fillStyle = color;
  ctx.font = "bold 9px sans-serif";
  ctx.fillText(label, sx + 6, sy - 4);
}

const MAP_VIEW_HEIGHT = 600;

/**
 * 공유 Occupancy 맵 — 여러 로봇 pose를 색으로 구분.
 * 드롭다운으로 선택한 로봇에만 좌(드래그 pose)·우(goal)·정지 적용.
 */
export function OccupancyNavMap({
  mapRobotId,
  robots,
  controlRobotId,
  onControlRobotChange,
  className,
}: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const viewportRef = useRef<HTMLDivElement>(null);
  const imgRef = useRef<HTMLImageElement | null>(null);
  const viewRef = useRef({
    scale: 1,
    offsetX: 0,
    offsetY: 0,
  });
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

  const mapQ = mapRobotId
    ? `?robot=${encodeURIComponent(mapRobotId)}`
    : "";
  const controlQ = controlRobotId
    ? `?robot=${encodeURIComponent(controlRobotId)}`
    : "";

  const controlRobot = useMemo(
    () => robots.find((r) => r.id === controlRobotId) || robots[0] || null,
    [robots, controlRobotId],
  );

  useEffect(() => {
    if (!mapRobotId) return;
    let revoked: string | null = null;
    let cancelled = false;
    (async () => {
      try {
        const m = await api<MapMeta>(`/admin/robot/map/meta${mapQ}`);
        if (cancelled) return;
        setMeta(m);
        const token = getToken();
        const res = await fetch(
          `${getApiUrl()}/admin/robot/map/image${mapQ}`,
          {
            headers: token ? { Authorization: `Bearer ${token}` } : {},
          },
        );
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
  }, [mapQ, mapRobotId]);

  const redraw = useCallback(() => {
    const canvas = canvasRef.current;
    const viewport = viewportRef.current;
    const img = imgRef.current;
    if (!canvas || !meta) return;
    const availW = Math.max(120, viewport?.clientWidth || 480);
    const availH = MAP_VIEW_HEIGHT;
    const scale = Math.min(availW / meta.width, availH / meta.height);
    const mapW = Math.round(meta.width * scale);
    const mapH = Math.round(meta.height * scale);
    const offsetX = (availW - mapW) / 2;
    const offsetY = (availH - mapH) / 2;
    viewRef.current = { scale, offsetX, offsetY };

    const dpr = window.devicePixelRatio || 1;
    canvas.width = availW * dpr;
    canvas.height = availH * dpr;
    canvas.style.width = `${availW}px`;
    canvas.style.height = `${availH}px`;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    ctx.fillStyle = "#e8e6e1";
    ctx.fillRect(0, 0, availW, availH);
    if (img) {
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(img, offsetX, offsetY, mapW, mapH);
    }

    const toScreen = (col: number, row: number) => ({
      sx: offsetX + col * scale,
      sy: offsetY + row * scale,
    });

    // 조종 대상 로봇 라이다만 (혼잡 방지)
    if (controlRobot?.pose && (controlRobot.lidarPoints?.length || 0) > 0) {
      const pose = controlRobot.pose;
      const c = Math.cos(pose.yaw);
      const s = Math.sin(pose.yaw);
      ctx.fillStyle = `${controlRobot.color}cc`;
      for (const p of controlRobot.lidarPoints || []) {
        if (!Number.isFinite(p.x) || !Number.isFinite(p.y)) continue;
        const lx = -p.x;
        const ly = -p.y;
        const mx = pose.x + c * lx - s * ly;
        const my = pose.y + s * lx + c * ly;
        const { col, row } = worldToPixel(meta, mx, my);
        const { sx, sy } = toScreen(col, row);
        ctx.beginPath();
        ctx.arc(sx, sy, 1.1, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    for (const robot of robots) {
      if (!robot.pose) continue;
      const { col, row } = worldToPixel(
        meta,
        robot.pose.x,
        robot.pose.y,
      );
      const { sx, sy } = toScreen(col, row);
      const isControl = robot.id === controlRobotId;
      drawRobotMarker(
        ctx,
        sx,
        sy,
        robot.pose.yaw,
        robot.color,
        Boolean(robot.navigating),
        robot.label,
      );
      if (isControl) {
        ctx.strokeStyle = robot.color;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(sx, sy, 10, 0, Math.PI * 2);
        ctx.stroke();
      }
    }

    const drag = dragRef.current;
    if (drag?.active) {
      const a = toScreen(drag.startCol, drag.startRow);
      const b = toScreen(drag.curCol, drag.curRow);
      const isGoal = drag.mode === "goal";
      const stroke =
        controlRobot?.color ||
        (isGoal ? "rgba(196, 92, 38, 0.95)" : "rgba(80, 100, 180, 0.9)");
      ctx.strokeStyle = stroke;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(a.sx, a.sy);
      ctx.lineTo(b.sx, b.sy);
      ctx.stroke();
      ctx.fillStyle = stroke;
      ctx.beginPath();
      ctx.arc(a.sx, a.sy, 3, 0, Math.PI * 2);
      ctx.fill();
      const ang = Math.atan2(b.sy - a.sy, b.sx - a.sx);
      ctx.beginPath();
      ctx.moveTo(b.sx, b.sy);
      ctx.lineTo(
        b.sx - 7 * Math.cos(ang - 0.4),
        b.sy - 7 * Math.sin(ang - 0.4),
      );
      ctx.lineTo(
        b.sx - 7 * Math.cos(ang + 0.4),
        b.sy - 7 * Math.sin(ang + 0.4),
      );
      ctx.closePath();
      ctx.fill();
    }

    // 범례
    let legendY = 14;
    ctx.font = "11px sans-serif";
    for (const robot of robots) {
      ctx.fillStyle = robot.color;
      ctx.fillRect(8, legendY - 7, 8, 8);
      ctx.fillStyle = "rgba(60,55,50,0.9)";
      ctx.fillText(
        `${robot.label}${robot.id === controlRobotId ? " · 조종" : ""}`,
        20,
        legendY,
      );
      legendY += 14;
    }

    ctx.fillStyle = "rgba(60,55,50,0.85)";
    ctx.font = "11px sans-serif";
    ctx.fillText(
      `${meta.mapId || "map"} · ${meta.width}×${meta.height} · ${meta.resolution}m`,
      8,
      availH - 8,
    );
    ctx.fillText("좌: pose · 우: goal (선택 로봇)", 8, availH - 22);
  }, [meta, robots, controlRobot, controlRobotId]);

  useEffect(() => {
    redraw();
  }, [redraw, mapUrl]);

  useEffect(() => {
    const el = viewportRef.current;
    if (!el || typeof ResizeObserver === "undefined") return;
    const ro = new ResizeObserver(() => {
      redraw();
    });
    ro.observe(el);
    return () => ro.disconnect();
  }, [redraw]);

  const eventToPixel = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    if (!canvas || !meta) return null;
    const rect = canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left;
    const sy = e.clientY - rect.top;
    const { scale, offsetX, offsetY } = viewRef.current;
    if (scale <= 0) return null;
    return {
      col: (sx - offsetX) / scale,
      row: (sy - offsetY) / scale,
    };
  };

  const finishDrag = async (
    mode: DragMode,
    startCol: number,
    startRow: number,
    curCol: number,
    curRow: number,
  ) => {
    if (!meta || busy || !controlRobotId) return;
    const start = pixelToWorld(meta, startCol, startRow);
    const end = pixelToWorld(meta, curCol, curRow);
    const dx = end.x - start.x;
    const dy = end.y - start.y;
    const yaw =
      Math.hypot(dx, dy) < meta.resolution * 0.5
        ? 0
        : Math.atan2(dy, dx);
    const body = { x: start.x, y: start.y, yaw };
    const who = controlRobot?.label || controlRobotId;
    setBusy(true);
    setStatus(null);
    try {
      if (mode === "pose") {
        const res = await api<{ success?: boolean; message?: string }>(
          `/admin/robot/nav/initialpose${controlQ}`,
          { method: "POST", body: JSON.stringify(body) },
        );
        if (!res.success) {
          setStatus(res.message || "initialpose 실패");
        } else {
          setStatus(
            `${who} pose (${start.x.toFixed(2)}, ${start.y.toFixed(2)}, yaw=${yaw.toFixed(2)})`,
          );
        }
      } else {
        const res = await api<{ success?: boolean; message?: string }>(
          `/admin/robot/nav/goal${controlQ}`,
          { method: "POST", body: JSON.stringify(body) },
        );
        if (!res.success) {
          setStatus(res.message || "Nav2 액션 서버 없음 / goal 실패");
        } else {
          setLastGoal(body);
          setStatus(
            `${who} goal (${start.x.toFixed(2)}, ${start.y.toFixed(2)}, yaw=${yaw.toFixed(2)})`,
          );
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
    if (!meta || busy || !controlRobotId) return;
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
    if (!controlRobotId) return;
    setBusy(true);
    try {
      const res = await api<{
        ok?: boolean;
        mission?: { orderId?: number } | null;
      }>(`/admin/robot/nav/stop${controlQ}`, {
        method: "POST",
        body: "{}",
      });
      const who = controlRobot?.label || controlRobotId;
      if (res.mission?.orderId) {
        setStatus(
          `${who} 정지 · 주문 #${res.mission.orderId} FAILED (그 자리 유지)`,
        );
      } else {
        setStatus(`${who} 정지 (할당 작업 없음)`);
      }
    } catch (err) {
      setStatus(err instanceof Error ? err.message : "stop 오류");
    } finally {
      setBusy(false);
    }
  };

  const onReturnHome = async () => {
    if (!controlRobotId) return;
    setBusy(true);
    try {
      const res = await api<{
        ok?: boolean;
        home?: { id?: string };
        alreadyReturning?: boolean;
        mission?: { orderId?: number } | null;
      }>(`/admin/robot/return-home${controlQ}`, {
        method: "POST",
        body: "{}",
      });
      const who = controlRobot?.label || controlRobotId;
      const homeId = res.home?.id || "홈";
      if (res.alreadyReturning) {
        setStatus(`${who} 이미 ${homeId} 복귀 중`);
      } else if (res.mission?.orderId) {
        setStatus(
          `${who} → ${homeId} 복귀 · 주문 #${res.mission.orderId} FAILED`,
        );
      } else {
        setStatus(`${who} → ${homeId} 복귀 시작`);
      }
    } catch (err) {
      setStatus(err instanceof Error ? err.message : "복귀 오류");
    } finally {
      setBusy(false);
    }
  };

  const pose = controlRobot?.pose;

  return (
    <div className={className || "occupancy-nav-map"}>
      <div className="occupancy-nav-toolbar">
        <label className="occupancy-control-select">
          <span className="muted">조종 로봇</span>
          <select
            value={controlRobotId}
            disabled={busy || robots.length === 0}
            onChange={(e) => onControlRobotChange?.(e.target.value)}
          >
            {robots.map((r) => (
              <option key={r.id} value={r.id}>
                {r.label}
              </option>
            ))}
          </select>
        </label>
        <button
          type="button"
          className="btn secondary"
          disabled={busy || !controlRobotId}
          onClick={() => void onStop()}
        >
          주행 정지
        </button>
        <button
          type="button"
          className="btn secondary"
          disabled={busy || !controlRobotId}
          onClick={() => void onReturnHome()}
        >
          복귀
        </button>
        <span className="muted" style={{ fontSize: "0.85rem" }}>
          {controlRobot?.navigating ? "주행 중" : "대기"}
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
      <div ref={viewportRef} className="occupancy-nav-viewport">
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
          style={{ cursor: "crosshair", touchAction: "none" }}
        />
      </div>      {status ? (
        <p className="muted" style={{ margin: "0.5rem 0 0", fontSize: "0.85rem" }}>
          {status}
        </p>
      ) : null}
    </div>
  );
}
