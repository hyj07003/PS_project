import "./load-env";
import "reflect-metadata";
import { NestFactory } from "@nestjs/core";
import { AppModule } from "./app.module";

async function bootstrap() {
  const app = await NestFactory.create(AppModule);
  app.enableCors({ origin: true });
  const port = Number(process.env.CONTROLLER_PORT || 4100);
  await app.listen(port, "127.0.0.1");
  console.log(`[controller] listening on http://127.0.0.1:${port}`);
}

bootstrap();
