/** @type {import('next').NextConfig} */
const nextConfig = {
  transpilePackages: ["@smartshop/shared"],
  // Allow phone / LAN access in next dev (cross-origin /_next assets)
  allowedDevOrigins: [
    "127.0.0.1",
    "localhost",
    "192.168.45.152",
    "192.168.*.*",
  ],
  async rewrites() {
    const api =
      process.env.WEB_SERVER_URL ||
      process.env.NEXT_PUBLIC_API_URL ||
      "http://127.0.0.1:4000";
    return [
      {
        source: "/uploads/:path*",
        destination: `${api}/uploads/:path*`,
      },
    ];
  },
  images: {
    remotePatterns: [
      { protocol: "https", hostname: "placehold.co" },
      { protocol: "http", hostname: "127.0.0.1" },
      { protocol: "http", hostname: "localhost" },
      { protocol: "http", hostname: "192.168.45.152" },
    ],
  },
};

module.exports = nextConfig;
