import "./load-env";
import "reflect-metadata";
import { NestFactory } from "@nestjs/core";
import { NestExpressApplication } from "@nestjs/platform-express";
import * as path from "path";
import * as fs from "fs";
import { AppModule } from "./app.module";

async function bootstrap() {
  const app = await NestFactory.create<NestExpressApplication>(AppModule);
  app.enableCors({
    origin: true,
    credentials: true,
  });

  const uploadDir = path.resolve(
    process.cwd(),
    process.env.UPLOAD_DIR || "./uploads",
  );
  fs.mkdirSync(uploadDir, { recursive: true });
  app.useStaticAssets(uploadDir, { prefix: "/uploads" });

  const port = Number(process.env.WEB_SERVER_PORT || 4000);
  const host = process.env.WEB_SERVER_HOST || "0.0.0.0";
  await app.listen(port, host);
  console.log(`[web-server] listening on http://${host}:${port}`);
}

bootstrap();
