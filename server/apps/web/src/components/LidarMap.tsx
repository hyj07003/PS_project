"use client";

import { useEffect, useRef } from "react";

export type LidarPoint = { x: number; y: number; r?: number };

type Props = {
  points?: LidarPoint[];
  rangeMax?: number;
  className?: string;
};

/**
 * 로봇 원점 기준 라이다 탑뷰(맵) 캔버스.
 * ROS LaserScan 관례: x 전방, y 좌측.
 */
export function LidarMap({ points = [], rangeMax = 8, className }: Props) {
  const ref = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = ref.current;
    if (!canvas) return;
    const parent = canvas.parentElement;
    const size = Math.min(parent?.clientWidth || 520, 560);
    const dpr = window.devicePixelRatio || 1;
    canvas.width = size * dpr;
    canvas.height = size * dpr;
    canvas.style.width = `${size}px`;
    canvas.style.height = `${size}px`;

    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    const cx = size / 2;
    const cy = size / 2;
    const maxR = Math.max(rangeMax, 1);
    const scale = (size * 0.42) / maxR;

    // background
    ctx.fillStyle = "rgba(255,255,255,0.55)";
    ctx.fillRect(0, 0, size, size);

    // grid rings
    ctx.strokeStyle = "rgba(44,44,44,0.12)";
    ctx.lineWidth = 1;
    for (let m = 1; m <= Math.ceil(maxR); m++) {
      ctx.beginPath();
      ctx.arc(cx, cy, m * scale, 0, Math.PI * 2);
      ctx.stroke();
    }

    // axes
    ctx.strokeStyle = "rgba(44,44,44,0.22)";
    ctx.beginPath();
    ctx.moveTo(0, cy);
    ctx.lineTo(size, cy);
    ctx.moveTo(cx, 0);
    ctx.lineTo(cx, size);
    ctx.stroke();

    // forward marker (+x up on screen: screenY = cy - x*scale)
    ctx.fillStyle = "#4f5a4b";
    ctx.beginPath();
    ctx.moveTo(cx, cy - 14);
    ctx.lineTo(cx - 7, cy + 6);
    ctx.lineTo(cx + 7, cy + 6);
    ctx.closePath();
    ctx.fill();

    // points
    ctx.fillStyle = "rgba(109,122,104,0.9)";
    for (const p of points) {
      if (!Number.isFinite(p.x) || !Number.isFinite(p.y)) continue;
      const sx = cx - p.y * scale; // y left → screen x
      const sy = cy - p.x * scale; // x forward → screen up
      ctx.beginPath();
      ctx.arc(sx, sy, 2.2, 0, Math.PI * 2);
      ctx.fill();
    }

    // robot body
    ctx.fillStyle = "#2c2c2c";
    ctx.beginPath();
    ctx.arc(cx, cy, 5, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = "rgba(90,87,82,0.9)";
    ctx.font = "12px sans-serif";
    ctx.fillText(`0…${maxR.toFixed(0)} m`, 10, size - 12);
    ctx.fillText(`${points.length} pts`, size - 64, size - 12);
  }, [points, rangeMax]);

  return (
    <div className={className || "lidar-map"}>
      <canvas ref={ref} aria-label="라이다 맵" />
    </div>
  );
}
