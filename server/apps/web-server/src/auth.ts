import {
  CanActivate,
  ExecutionContext,
  Inject,
  Injectable,
  UnauthorizedException,
  ForbiddenException,
  createParamDecorator,
} from "@nestjs/common";
import { JwtService } from "@nestjs/jwt";
import type { UserRole } from "@smartshop/shared";

export type AuthPayload = {
  sub: number;
  email: string;
  role: UserRole;
};

function readToken(req: { headers: { authorization?: string } }): string {
  const header = req.headers.authorization;
  if (!header?.startsWith("Bearer ")) {
    throw new UnauthorizedException("missing token");
  }
  return header.slice(7);
}

@Injectable()
export class AuthGuard implements CanActivate {
  constructor(@Inject(JwtService) private readonly jwt: JwtService) {}

  canActivate(context: ExecutionContext): boolean {
    const req = context.switchToHttp().getRequest();
    try {
      req.user = this.jwt.verify<AuthPayload>(readToken(req));
      return true;
    } catch (err) {
      if (err instanceof UnauthorizedException) throw err;
      throw new UnauthorizedException("invalid token");
    }
  }
}

@Injectable()
export class AdminGuard implements CanActivate {
  constructor(@Inject(JwtService) private readonly jwt: JwtService) {}

  canActivate(context: ExecutionContext): boolean {
    const req = context.switchToHttp().getRequest();
    try {
      const payload = this.jwt.verify<AuthPayload>(readToken(req));
      req.user = payload;
      if (payload.role !== "admin") {
        throw new ForbiddenException("admin only");
      }
      return true;
    } catch (err) {
      if (
        err instanceof UnauthorizedException ||
        err instanceof ForbiddenException
      ) {
        throw err;
      }
      throw new UnauthorizedException("invalid token");
    }
  }
}

export const CurrentUser = createParamDecorator(
  (_data: unknown, ctx: ExecutionContext): AuthPayload => {
    return ctx.switchToHttp().getRequest().user;
  },
);
