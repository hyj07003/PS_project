"use client";

import Link from "next/link";
import { FormEvent, useState } from "react";
import { useRouter } from "next/navigation";
import { useAuth } from "@/lib/auth-context";

export default function LoginPage() {
  const { login } = useAuth();
  const router = useRouter();
  const [email, setEmail] = useState("customer@smartshop.local");
  const [password, setPassword] = useState("customer1234");
  const [error, setError] = useState<string | null>(null);
  const [pending, setPending] = useState(false);

  const onSubmit = async (e: FormEvent) => {
    e.preventDefault();
    setPending(true);
    setError(null);
    try {
      await login(email, password);
      router.push("/");
    } catch (err) {
      setError(err instanceof Error ? err.message : "로그인 실패");
    } finally {
      setPending(false);
    }
  };

  return (
    <div className="container page">
      <h1 className="hero-title" style={{ fontSize: "2.4rem" }}>
        로그인
      </h1>
      <p className="muted">
        데모 계정: customer@smartshop.local / customer1234 · admin@smartshop.local
        / admin1234
      </p>
      <form className="form" onSubmit={onSubmit}>
        <label>
          이메일
          <input value={email} onChange={(e) => setEmail(e.target.value)} />
        </label>
        <label>
          비밀번호
          <input
            type="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
          />
        </label>
        {error ? <p className="error">{error}</p> : null}
        <button className="btn" type="submit" disabled={pending}>
          {pending ? "처리 중…" : "로그인"}
        </button>
      </form>
      <p>
        계정이 없나요? <Link href="/register">회원가입</Link>
      </p>
    </div>
  );
}
