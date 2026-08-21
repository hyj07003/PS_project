import {
  Body,
  Controller,
  Delete,
  Get,
  HttpException,
  Inject,
  Param,
  Patch,
  Post,
  Put,
  Query,
  Res,
  UploadedFile,
  UseGuards,
  UseInterceptors,
} from "@nestjs/common";
import { FileInterceptor } from "@nestjs/platform-express";
import { JwtService } from "@nestjs/jwt";
import type { Response } from "express";
import type {
  AuthLoginInput,
  AuthRegisterInput,
  AuthResponse,
  CreateProductInput,
  GuestCartItem,
  UpdateProductInput,
  UserPublic,
} from "@smartshop/shared";
import * as fs from "fs";
import * as path from "path";
import { diskStorage } from "multer";
import { AdminGuard, AuthGuard, CurrentUser, type AuthPayload } from "./auth";
import { controllerJson } from "./controller-client";
import {
  homePoseForDevice,
  listPinkyRobots,
  pinkyBinary,
  pinkyJson,
  resolvePinkyUrl,
} from "./pinky-client";

function parseNavPath(raw: unknown): { x: number; y: number }[] {
  if (!raw || typeof raw !== "object") return [];
  const obj = raw as Record<string, unknown>;
  let poses: unknown = obj.poses;
  if (!Array.isArray(poses) && Array.isArray(obj.path)) {
    poses = obj.path;
  }
  if (!Array.isArray(poses) && obj.path && typeof obj.path === "object") {
    poses = (obj.path as { poses?: unknown }).poses;
  }
  if (!Array.isArray(poses)) return [];
  const out: { x: number; y: number }[] = [];
  for (const item of poses) {
    if (!item || typeof item !== "object") continue;
    const x = (item as { x?: unknown }).x;
    const y = (item as { y?: unknown }).y;
    if (typeof x === "number" && typeof y === "number" && Number.isFinite(x) && Number.isFinite(y)) {
      out.push({ x, y });
    }
  }
  return out;
}

function asNavPose(raw: unknown): { x: number; y: number; yaw: number } | null {
  if (!raw || typeof raw !== "object") return null;
  const o = raw as { x?: unknown; y?: unknown; yaw?: unknown };
  if (
    typeof o.x !== "number" ||
    typeof o.y !== "number" ||
    !Number.isFinite(o.x) ||
    !Number.isFinite(o.y)
  ) {
    return null;
  }
  return {
    x: o.x,
    y: o.y,
    yaw: typeof o.yaw === "number" && Number.isFinite(o.yaw) ? o.yaw : 0,
  };
}

function wrapError(err: unknown): never {
  const e = err as Error & { status?: number };
  throw new HttpException(e.message || "upstream error", e.status || 502);
}

function normalizeDeviceCode(code: string | undefined | null): string {
  const c = (code || "").trim().toLowerCase();
  if (c === "cart-2" || c === "cart2" || c === "2") return "cart-2";
  if (c === "cart-1" || c === "cart1" || c === "1") return "cart-1";
  return c;
}

/** DEVICE_CODE mismatch 배지만. 홈 initialpose 자동 적용 금지
 *  (작업 중 멈칫/폴링 때 대기장소 점프 원인). */
async function ensureRobotHomePose(opts: {
  robotId: string;
  url: string;
  reportedDeviceCode?: string | null;
  pose?: { x: number; y: number; yaw: number } | null;
  navigating?: boolean;
  hasActiveAssignment?: boolean;
}): Promise<{ synced: boolean; mismatch: boolean }> {
  const expected = normalizeDeviceCode(opts.robotId);
  const reported = normalizeDeviceCode(opts.reportedDeviceCode);
  const mismatch = Boolean(reported && expected && reported !== expected);
  return { synced: false, mismatch };
}

@Controller()
export class AppController {
  constructor(@Inject(JwtService) private readonly jwt: JwtService) {}

  @Get("health")
  health() {
    return { ok: true, service: "web-server" };
  }

  @Post("auth/register")
  async register(@Body() body: AuthRegisterInput): Promise<AuthResponse> {
    try {
      const user = await controllerJson<UserPublic>("/users/register", {
        method: "POST",
        body: JSON.stringify(body),
      });
      const token = await this.jwt.signAsync({
        sub: user.id,
        email: user.email,
        role: user.role,
      });
      return { token, user };
    } catch (err) {
      wrapError(err);
    }
  }

  @Post("auth/login")
  async login(@Body() body: AuthLoginInput): Promise<AuthResponse> {
    try {
      const user = await controllerJson<UserPublic>("/users/login", {
        method: "POST",
        body: JSON.stringify(body),
      });
      const token = await this.jwt.signAsync({
        sub: user.id,
        email: user.email,
        role: user.role,
      });
      return { token, user };
    } catch (err) {
      wrapError(err);
    }
  }

  @Get("auth/me")
  @UseGuards(AuthGuard)
  async me(@CurrentUser() user: AuthPayload) {
    try {
      return await controllerJson<UserPublic>(`/users/${user.sub}`);
    } catch (err) {
      wrapError(err);
    }
  }

  @Get("categories")
  async categories() {
    try {
      return await controllerJson("/categories");
    } catch (err) {
      wrapError(err);
    }
  }

  @Get("products")
  async products(
    @Query("q") q?: string,
    @Query("category") category?: string,
    @Query("featured") featured?: string,
  ) {
    try {
      const params = new URLSearchParams();
      if (q) params.set("q", q);
      if (category) params.set("category", category);
      if (featured) params.set("featured", featured);
      const qs = params.toString();
      return await controllerJson(`/products${qs ? `?${qs}` : ""}`);
    } catch (err) {
      wrapError(err);
    }
  }

  @Get("products/:id")
  async product(@Param("id") id: string) {
    try {
      return await controllerJson(`/products/${id}?activeOnly=1`);
    } catch (err) {
      wrapError(err);
    }
  }

  @Get("cart")
  @UseGuards(AuthGuard)
  async getCart(@CurrentUser() user: AuthPayload) {
    try {
      return await controllerJson(`/carts/${user.sub}`);
    } catch (err) {
      wrapError(err);
    }
  }

  @Post("cart/items")
  @UseGuards(AuthGuard)
  async addCartItem(
    @CurrentUser() user: AuthPayload,
    @Body() body: { productId: number; quantity?: number },
  ) {
    try {
      return await controllerJson(`/carts/${user.sub}/items`, {
        method: "POST",
        body: JSON.stringify(body),
      });
    } catch (err) {
      wrapError(err);
    }
  }

  @Patch("cart/items/:productId")
  @UseGuards(AuthGuard)
  async updateCartItem(
    @CurrentUser() user: AuthPayload,
    @Param("productId") productId: string,
    @Body() body: { quantity: number },
  ) {
    try {
      return await controllerJson(
        `/carts/${user.sub}/items/${productId}`,
        {
          method: "PATCH",
          body: JSON.stringify(body),
        },
      );
    } catch (err) {
      wrapError(err);
    }
  }

  @Delete("cart/items/:productId")
  @UseGuards(AuthGuard)
  async removeCartItem(
    @CurrentUser() user: AuthPayload,
    @Param("productId") productId: string,
  ) {
    try {
      return await controllerJson(
        `/carts/${user.sub}/items/${productId}`,
        { method: "DELETE" },
      );
    } catch (err) {
      wrapError(err);
    }
  }

  @Post("cart/merge")
  @UseGuards(AuthGuard)
  async mergeCart(
    @CurrentUser() user: AuthPayload,
    @Body() body: { items: GuestCartItem[] },
  ) {
    try {
      return await controllerJson(`/carts/${user.sub}/merge`, {
        method: "POST",
        body: JSON.stringify(body),
      });
    } catch (err) {
      wrapError(err);
    }
  }

  @Post("orders")
  @UseGuards(AuthGuard)
  async createOrder(@CurrentUser() user: AuthPayload) {
    try {
      return await controllerJson(`/orders`, {
        method: "POST",
        body: JSON.stringify({ userId: user.sub }),
      });
    } catch (err) {
      wrapError(err);
    }
  }

  @Get("orders/:id")
  @UseGuards(AuthGuard)
  async getOrder(@Param("id") id: string) {
    try {
      return await controllerJson(`/orders/${id}`);
    } catch (err) {
      wrapError(err);
    }
  }

  @Get("admin/products")
  @UseGuards(AdminGuard)
  async adminProducts() {
    try {
      return await controllerJson(`/products?includeInactive=1`);
    } catch (err) {
      wrapError(err);
    }
  }

  @Post("admin/products")
  @UseGuards(AdminGuard)
  async adminCreate(
    @CurrentUser() user: AuthPayload,
    @Body() body: CreateProductInput,
  ) {
    try {
      return await controllerJson(`/products`, {
        method: "POST",
        body: JSON.stringify({ ...body, createdBy: user.sub }),
      });
    } catch (err) {
      wrapError(err);
    }
  }

  @Post("admin/products/reset-stock")
  @UseGuards(AdminGuard)
  async adminResetStock() {
    try {
      return await controllerJson(`/products/reset-stock`, { method: "POST" });
    } catch (err) {
      wrapError(err);
    }
  }

  @Put("admin/products/:id")
  @UseGuards(AdminGuard)
  async adminUpdate(
    @Param("id") id: string,
    @Body() body: UpdateProductInput,
  ) {
    try {
      return await controllerJson(`/products/${id}`, {
        method: "PUT",
        body: JSON.stringify(body),
      });
    } catch (err) {
      wrapError(err);
    }
  }

  @Delete("admin/products/:id")
  @UseGuards(AdminGuard)
  async adminDelete(@Param("id") id: string) {
    try {
      return await controllerJson(`/products/${id}`, { method: "DELETE" });
    } catch (err) {
      wrapError(err);
    }
  }

  @Post("admin/upload")
  @UseGuards(AdminGuard)
  @UseInterceptors(
    FileInterceptor("file", {
      storage: diskStorage({
        destination: (_req, _file, cb) => {
          const dir = path.resolve(
            process.cwd(),
            process.env.UPLOAD_DIR || "./uploads",
          );
          fs.mkdirSync(dir, { recursive: true });
          cb(null, dir);
        },
        filename: (_req, file, cb) => {
          const safe = file.originalname.replace(/[^\w.\-]+/g, "_");
          cb(null, `${Date.now()}-${safe}`);
        },
      }),
      limits: { fileSize: 5 * 1024 * 1024 },
    }),
  )
  upload(@UploadedFile() file: Express.Multer.File) {
    if (!file) throw new HttpException("file required", 400);
    // Relative path — clients resolve with current host (LAN / phone safe)
    return { url: `/uploads/${file.filename}` };
  }

  /** 등록된 주행로봇 목록 + 각 대 센서 스냅샷 (세로 패널용) */
  @Get("admin/robots")
  @UseGuards(AdminGuard)
  async robotsMonitor() {
    const targets = listPinkyRobots();
    let queueLength = 0;
    try {
      const q = await controllerJson<{ queueLength?: number }>(
        "/missions/queue",
      );
      queueLength = q.queueLength ?? 0;
    } catch {
      queueLength = 0;
    }

    const robots = await Promise.all(
      targets.map(async (t) => {
        let assignment: unknown = null;
        try {
          const missions = await controllerJson<unknown[]>(
            `/missions?deviceCode=${encodeURIComponent(t.id)}&active=1&includeOrder=1`,
          );
          assignment =
            Array.isArray(missions) && missions.length > 0 ? missions[0] : null;
        } catch {
          assignment = null;
        }

        try {
          const [health, sensors, navRaw, pathRaw] = await Promise.all([
            pinkyJson("/health", undefined, t.url),
            pinkyJson("/sensors", undefined, t.url),
            pinkyJson("/nav/state", undefined, t.url).catch(() => null),
            pinkyJson("/nav/path", undefined, t.url).catch(() => null),
          ]);
          const home = homePoseForDevice(t.id);
          const navObj =
            navRaw && typeof navRaw === "object"
              ? (navRaw as Record<string, unknown>)
              : null;
          const navPose =
            navObj &&
            navObj.pose &&
            typeof navObj.pose === "object" &&
            typeof (navObj.pose as { x?: unknown }).x === "number"
              ? (navObj.pose as { x: number; y: number; yaw: number })
              : null;
          const sensorsObj =
            sensors && typeof sensors === "object"
              ? (sensors as Record<string, unknown>)
              : null;
          const sensorPose =
            sensorsObj &&
            sensorsObj.pose &&
            typeof sensorsObj.pose === "object" &&
            typeof (sensorsObj.pose as { x?: unknown }).x === "number"
              ? (sensorsObj.pose as { x: number; y: number; yaw: number })
              : null;
          const healthObj =
            health && typeof health === "object"
              ? (health as Record<string, unknown>)
              : null;
          const reportedCode =
            (typeof healthObj?.deviceCode === "string"
              ? healthObj.deviceCode
              : null) ||
            (typeof sensorsObj?.deviceCode === "string"
              ? sensorsObj.deviceCode
              : null);
          const activeStatuses = new Set([
            "ASSIGNED",
            "PICKING",
            "CHECKOUT",
            "PACKING",
            "RETURNING",
          ]);
          const assignmentObj =
            assignment && typeof assignment === "object"
              ? (assignment as {
                  status?: string;
                  currentWaypointPose?: unknown;
                })
              : null;
          const hasActiveAssignment = Boolean(
            assignmentObj?.status && activeStatuses.has(assignmentObj.status),
          );
          const livePose = navPose || sensorPose || null;
          const homeFix = await ensureRobotHomePose({
            robotId: t.id,
            url: t.url,
            reportedDeviceCode: reportedCode,
            pose: livePose,
            navigating: Boolean(navObj?.navigating),
            hasActiveAssignment,
          });
          // 실제 로봇 pose 우선 — mismatch만으로 모니터를 홈으로 덮지 않음
          const pose = homeFix.synced ? home : livePose || home;
          const actionState = String(
            (navObj?.navigationAction as { state?: unknown } | undefined)?.state || "",
          ).toUpperCase();
          const navigating = Boolean(
            navObj?.navigating ||
              actionState === "ACCEPTED" ||
              actionState === "EXECUTING" ||
              actionState === "CANCELING",
          );
          const pathFromState = parseNavPath(navObj);
          const pathPoints = parseNavPath(pathRaw);
          let path =
            navigating && pathPoints.length > 1
              ? pathPoints
              : navigating
                ? pathFromState
                : [];
          const navGoal = navigating ? asNavPose(navObj?.goal) : null;
          const assignmentGoal = hasActiveAssignment
            ? asNavPose(assignmentObj?.currentWaypointPose)
            : null;
          const goal = navGoal || assignmentGoal;
          if (
            navigating &&
            path.length < 2 &&
            pose &&
            goal &&
            Number.isFinite(pose.x) &&
            Number.isFinite(goal.x)
          ) {
            const sameXy =
              Math.hypot(goal.x - pose.x, goal.y - pose.y) < 0.05;
            path = sameXy
              ? [
                  { x: pose.x, y: pose.y },
                  {
                    x: pose.x + 0.35 * Math.cos(goal.yaw),
                    y: pose.y + 0.35 * Math.sin(goal.yaw),
                  },
                ]
              : [
                  { x: pose.x, y: pose.y },
                  { x: goal.x, y: goal.y },
                ];
          }
          const nav = {
            ...(navObj || {}),
            pose,
            mapId: (navObj?.mapId as string | null | undefined) ?? null,
            navigating,
            path,
            goal,
            expectedHome: home,
            deviceCodeMismatch: homeFix.mismatch,
            homeSynced: homeFix.synced,
          };
          const sensorsWithPose = {
            ...(sensorsObj || {}),
            pose,
          };
          return {
            id: t.id,
            label: t.label,
            url: t.url,
            online: true,
            health,
            sensors: sensorsWithPose,
            nav,
            assignment,
            error: homeFix.mismatch
              ? `DEVICE_CODE 불일치(로봇=${reportedCode || "?"}, 기대=${t.id}) — S2/S1 홈 강제 중. pinky.env 확인`
              : (null as string | null),
          };
        } catch (err) {
          const e = err as Error;
          const home = homePoseForDevice(t.id);
          return {
            id: t.id,
            label: t.label,
            url: t.url,
            online: false,
            health: null,
            sensors: { pose: home },
            nav: { pose: home, navigating: false, mapId: null },
            assignment,
            error: e.message || "unreachable",
          };
        }
      }),
    );
    return { robots, count: robots.length, queueLength };
  }

  @Get("admin/robot/missions")
  @UseGuards(AdminGuard)
  async robotMissions(
    @Query("robot") robot?: string,
    @Query("active") active?: string,
  ) {
    try {
      const qs = new URLSearchParams();
      if (robot) qs.set("deviceCode", robot);
      if (active) qs.set("active", active);
      qs.set("includeOrder", "1");
      const q = qs.toString();
      return await controllerJson(`/missions${q ? `?${q}` : ""}`);
    } catch (err) {
      wrapError(err);
    }
  }

  @Get("admin/robot/missions/:id")
  @UseGuards(AdminGuard)
  async robotMissionDetail(@Param("id") id: string) {
    try {
      return await controllerJson(`/missions/${encodeURIComponent(id)}`);
    } catch (err) {
      wrapError(err);
    }
  }

  @Get("admin/robot/map/meta")
  @UseGuards(AdminGuard)
  async robotMapMeta(@Query("robot") robot?: string) {
    try {
      return await pinkyJson("/map/meta", undefined, resolvePinkyUrl(robot));
    } catch (err) {
      wrapError(err);
    }
  }

  @Get("admin/robot/map/image")
  @UseGuards(AdminGuard)
  async robotMapImage(
    @Query("robot") robot: string | undefined,
    @Res() res: Response,
  ) {
    try {
      const { buffer, contentType } = await pinkyBinary(
        "/map/image",
        resolvePinkyUrl(robot),
      );
      res.setHeader("Content-Type", contentType);
      res.setHeader("Cache-Control", "public, max-age=60");
      res.send(buffer);
    } catch (err) {
      wrapError(err);
    }
  }

  @Get("admin/robot/nav/path")
  @UseGuards(AdminGuard)
  async robotNavPath(@Query("robot") robot?: string) {
    try {
      return await pinkyJson("/nav/path", undefined, resolvePinkyUrl(robot));
    } catch (err) {
      wrapError(err);
    }
  }

  @Get("admin/robot/nav/state")
  @UseGuards(AdminGuard)
  async robotNavState(@Query("robot") robot?: string) {
    try {
      return await pinkyJson("/nav/state", undefined, resolvePinkyUrl(robot));
    } catch (err) {
      wrapError(err);
    }
  }

  @Post("admin/robot/nav/initialpose")
  @UseGuards(AdminGuard)
  async robotNavInitialPose(
    @Body() body: Record<string, unknown>,
    @Query("robot") robot?: string,
  ) {
    try {
      return await pinkyJson(
        "/nav/initialpose",
        { method: "POST", body: JSON.stringify(body) },
        resolvePinkyUrl(robot),
      );
    } catch (err) {
      wrapError(err);
    }
  }

  @Post("admin/robot/nav/goal")
  @UseGuards(AdminGuard)
  async robotNavGoal(
    @Body() body: Record<string, unknown>,
    @Query("robot") robot?: string,
  ) {
    try {
      return await pinkyJson(
        "/nav/goal",
        { method: "POST", body: JSON.stringify(body) },
        resolvePinkyUrl(robot),
      );
    } catch (err) {
      wrapError(err);
    }
  }

  @Post("admin/robot/nav/stop")
  @UseGuards(AdminGuard)
  async robotNavStop(@Query("robot") robot?: string) {
    try {
      const code = robot || listPinkyRobots()[0]?.id || "cart-1";
      // 컨트롤러: 미션 FAILED + 자리 정지, Pinky stop 포함
      return await controllerJson(`/devices/${encodeURIComponent(code)}/abort`, {
        method: "POST",
        body: "{}",
      });
    } catch (err) {
      wrapError(err);
    }
  }

  @Post("admin/robot/return-home")
  @UseGuards(AdminGuard)
  async robotReturnHome(@Query("robot") robot?: string) {
    try {
      const code = robot || listPinkyRobots()[0]?.id || "cart-1";
      return await controllerJson(
        `/devices/${encodeURIComponent(code)}/return-home`,
        { method: "POST", body: "{}" },
      );
    } catch (err) {
      wrapError(err);
    }
  }

  @Get("admin/robot/health")
  @UseGuards(AdminGuard)
  async robotHealth(@Query("robot") robot?: string) {
    try {
      return await pinkyJson("/health", undefined, resolvePinkyUrl(robot));
    } catch (err) {
      wrapError(err);
    }
  }

  @Get("admin/robot/sensors")
  @UseGuards(AdminGuard)
  async robotSensors(@Query("robot") robot?: string) {
    try {
      return await pinkyJson("/sensors", undefined, resolvePinkyUrl(robot));
    } catch (err) {
      wrapError(err);
    }
  }

  @Get("admin/robot/sensors/:kind")
  @UseGuards(AdminGuard)
  async robotSensorKind(
    @Param("kind") kind: string,
    @Query("robot") robot?: string,
  ) {
    const allowed = new Set(["battery", "lidar", "imu", "ultrasonic"]);
    if (!allowed.has(kind)) {
      throw new HttpException("unknown sensor kind", 400);
    }
    try {
      return await pinkyJson(
        `/sensors/${kind}`,
        undefined,
        resolvePinkyUrl(robot),
      );
    } catch (err) {
      wrapError(err);
    }
  }

  @Get("admin/robot/devices")
  @UseGuards(AdminGuard)
  async robotDevices() {
    try {
      return await controllerJson("/devices");
    } catch (err) {
      wrapError(err);
    }
  }

  @Get("admin/robot/telemetry")
  @UseGuards(AdminGuard)
  async robotTelemetry() {
    try {
      return await controllerJson("/robot/telemetry");
    } catch (err) {
      wrapError(err);
    }
  }
}
