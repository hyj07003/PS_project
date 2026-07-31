import "./load-env";
import { Module } from "@nestjs/common";
import { JwtModule } from "@nestjs/jwt";
import { AppController } from "./app.controller";
import { AdminGuard, AuthGuard } from "./auth";

@Module({
  imports: [
    JwtModule.register({
      global: true,
      secret: process.env.JWT_SECRET || "smartshop-dev-secret-change-me",
      signOptions: { expiresIn: "7d" },
    }),
  ],
  controllers: [AppController],
  providers: [AuthGuard, AdminGuard],
})
export class AppModule {}
