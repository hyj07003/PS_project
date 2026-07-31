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
  UploadedFile,
  UseGuards,
  UseInterceptors,
} from "@nestjs/common";
import { FileInterceptor } from "@nestjs/platform-express";
import { JwtService } from "@nestjs/jwt";
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

function wrapError(err: unknown): never {
  const e = err as Error & { status?: number };
  throw new HttpException(e.message || "upstream error", e.status || 502);
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
}
