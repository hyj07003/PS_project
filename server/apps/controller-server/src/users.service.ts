import {
  BadRequestException,
  Inject,
  Injectable,
  UnauthorizedException,
} from "@nestjs/common";
import * as bcrypt from "bcryptjs";
import type { UserPublic, UserRole } from "@smartshop/shared";
import { DatabaseService } from "./database.service";
import { nowIso } from "./db/mappers";

type UserRow = {
  id: number;
  email: string;
  password_hash: string;
  name: string;
  role: UserRole;
  status: "active" | "disabled";
  created_at: string;
};

@Injectable()
export class UsersService {
  constructor(@Inject(DatabaseService) private readonly db: DatabaseService) {}

  private toPublic(row: UserRow): UserPublic {
    return {
      id: row.id,
      email: row.email,
      name: row.name,
      role: row.role,
      status: row.status,
      createdAt: row.created_at,
    };
  }

  findByEmail(email: string): UserRow | undefined {
    return this.db.connection
      .prepare("SELECT * FROM users WHERE email = ?")
      .get(email.toLowerCase()) as UserRow | undefined;
  }

  findById(id: number): UserPublic | null {
    const row = this.db.connection
      .prepare("SELECT * FROM users WHERE id = ?")
      .get(id) as UserRow | undefined;
    return row ? this.toPublic(row) : null;
  }

  register(email: string, password: string, name: string): UserPublic {
    if (!email || !password || !name) {
      throw new BadRequestException("email, password, name are required");
    }
    if (password.length < 6) {
      throw new BadRequestException("password must be at least 6 characters");
    }
    const existing = this.findByEmail(email);
    if (existing) {
      throw new BadRequestException("email already registered");
    }
    const ts = nowIso();
    const hash = bcrypt.hashSync(password, 10);
    const result = this.db.connection
      .prepare(
        `INSERT INTO users (email, password_hash, name, role, status, created_at, updated_at)
         VALUES (?, ?, ?, 'customer', 'active', ?, ?)`,
      )
      .run(email.toLowerCase(), hash, name, ts, ts);

    this.db.connection
      .prepare(`INSERT INTO carts (user_id, updated_at) VALUES (?, ?)`)
      .run(result.lastInsertRowid, ts);

    return this.findById(Number(result.lastInsertRowid))!;
  }

  verifyLogin(email: string, password: string): UserPublic {
    const row = this.findByEmail(email);
    if (!row || row.status !== "active") {
      throw new UnauthorizedException("invalid credentials");
    }
    if (!bcrypt.compareSync(password, row.password_hash)) {
      throw new UnauthorizedException("invalid credentials");
    }
    return this.toPublic(row);
  }
}
