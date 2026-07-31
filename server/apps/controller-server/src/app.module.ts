import { Module } from "@nestjs/common";
import { AppController } from "./app.controller";
import { CartsService } from "./carts.service";
import { DatabaseService } from "./database.service";
import { OrdersService } from "./orders.service";
import { ProductsService } from "./products.service";
import { UsersService } from "./users.service";

@Module({
  controllers: [AppController],
  providers: [
    DatabaseService,
    UsersService,
    ProductsService,
    CartsService,
    OrdersService,
  ],
})
export class AppModule {}
