import react from '@vitejs/plugin-react';
import { defineConfig } from 'vitest/config';

const CLOUDFLARE_ANALYTICS_URL = 'https://static.cloudflareinsights.com/beacon.min.js';
const CLOUDFLARE_ANALYTICS_TOKEN = 'cf409fe6f5e944d1ae674a7c10a6e082';

export default defineConfig({
  // GitHub Pages passes /UNSWMicromouseDemo/ here. Keeping '/' as the default
  // means localhost and the Docker/Nginx image continue serving from the root.
  base: process.env.VITE_BASE_PATH ?? '/',
  plugins: [
    react(),
    {
      name: 'cloudflare-web-analytics',
      apply: 'build',
      transformIndexHtml: {
        order: 'post',
        handler: () => [
          {
            tag: 'script',
            attrs: {
              type: 'module',
              src: CLOUDFLARE_ANALYTICS_URL,
              'data-cf-beacon': JSON.stringify({ token: CLOUDFLARE_ANALYTICS_TOKEN }),
            },
            injectTo: 'body',
          },
        ],
      },
    },
  ],
  server: {
    port: 5173,
  },
  preview: {
    port: 4173,
  },
  test: {
    exclude: ['tests/e2e/**', 'node_modules/**', 'dist/**'],
  },
});
