import {
  Body,
  Controller,
  Delete,
  Get,
  Inject,
  Param,
  Patch,
  Post,
  Put,
  Query,
} from "@nestjs/common";
import type {
  CreateProductInput,
  GuestCartItem,
  UpdateProductInput,
} from "@smartshop/shared";
import { CartsService } from "./carts.service";
import { OrdersService } from "./orders.service";
import { ProductsService } from "./products.service";
import { UsersService } from "./users.service";

@Controller()
export class AppController {
  constructor(
    @Inject(UsersService) private readonly users: UsersService,
    @Inject(ProductsService) private readonly products: ProductsService,
    @Inject(CartsService) private readonly carts: CartsService,
    @Inject(OrdersService) private readonly orders: OrdersService,
  ) {}

  @Get("health")
  health() {
    return { ok: true, service: "controller-server" };
  }

  @Post("users/register")
  register(
    @Body() body: { email: string; password: string; name: string },
  ) {
    return this.users.register(body.email, body.password, body.name);
  }

  @Post("users/login")
  login(@Body() body: { email: string; password: string }) {
    return this.users.verifyLogin(body.email, body.password);
  }

  @Get("users/:id")
  getUser(@Param("id") id: string) {
    return this.users.findById(Number(id));
  }

  @Get("categories")
  categories() {
    return this.products.listCategories();
  }

  @Get("products")
  listProducts(
    @Query("q") q?: string,
    @Query("category") category?: string,
    @Query("featured") featured?: string,
    @Query("includeInactive") includeInactive?: string,
  ) {
    return this.products.list({
      q,
      category,
      featured: featured === "1" || featured === "true",
      includeInactive: includeInactive === "1" || includeInactive === "true",
    });
  }

  @Get("products/:id")
  getProduct(
    @Param("id") id: string,
    @Query("activeOnly") activeOnly?: string,
  ) {
    return this.products.getById(
      Number(id),
      activeOnly === "1" || activeOnly === "true",
    );
  }

  @Post("products")
  createProduct(
    @Body() body: CreateProductInput & { createdBy?: number },
  ) {
    const { createdBy, ...input } = body;
    return this.products.create(input, createdBy);
  }

  @Put("products/:id")
  @Patch("products/:id")
  updateProduct(@Param("id") id: string, @Body() body: UpdateProductInput) {
    return this.products.update(Number(id), body);
  }

  @Delete("products/:id")
  deleteProduct(@Param("id") id: string) {
    return this.products.remove(Number(id));
  }

  @Get("carts/:userId")
  getCart(@Param("userId") userId: string) {
    return this.carts.getCart(Number(userId));
  }

  @Post("carts/:userId/items")
  addCartItem(
    @Param("userId") userId: string,
    @Body() body: { productId: number; quantity?: number },
  ) {
    return this.carts.addItem(
      Number(userId),
      body.productId,
      body.quantity ?? 1,
    );
  }

  @Patch("carts/:userId/items/:productId")
  updateCartItem(
    @Param("userId") userId: string,
    @Param("productId") productId: string,
    @Body() body: { quantity: number },
  ) {
    return this.carts.updateItem(
      Number(userId),
      Number(productId),
      body.quantity,
    );
  }

  @Delete("carts/:userId/items/:productId")
  removeCartItem(
    @Param("userId") userId: string,
    @Param("productId") productId: string,
  ) {
    return this.carts.removeItem(Number(userId), Number(productId));
  }

  @Post("carts/:userId/merge")
  mergeCart(
    @Param("userId") userId: string,
    @Body() body: { items: GuestCartItem[] },
  ) {
    return this.carts.mergeGuest(Number(userId), body.items ?? []);
  }

  @Post("orders")
  createOrder(@Body() body: { userId: number }) {
    return this.orders.createFromCart(body.userId);
  }

  @Get("orders/:id")
  getOrder(@Param("id") id: string) {
    return this.orders.getById(Number(id));
  }

  @Get("devices")
  devices() {
    return this.orders.listDevices();
  }
}
