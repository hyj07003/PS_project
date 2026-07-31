import { Injectable, OnModuleInit } from "@nestjs/common";
import type { DatabaseSync } from "node:sqlite";
import { migrate, openDatabase } from "./db/database";
import { seedIfEmpty } from "./db/seed";

@Injectable()
export class DatabaseService implements OnModuleInit {
  private db!: DatabaseSync;

  onModuleInit() {
    this.db = openDatabase();
    migrate(this.db);
    seedIfEmpty(this.db);
  }

  get connection(): DatabaseSync {
    return this.db;
  }
}
