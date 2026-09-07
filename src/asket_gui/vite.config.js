import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// The build output goes straight into gui_backend's static directory: one
// process serves the API and the frontend, because there is no CDN in Namibia
// and no second server to run on the Jetson.
export default defineConfig({
  plugins: [react()],
  build: {
    outDir: '../gui_backend/gui_backend/static',
    emptyOutDir: true,
    // Everything must load from the Jetson. No dynamic imports from a CDN, and
    // no source maps shipped to a laptop over a 4G link.
    sourcemap: false,
    target: 'es2022',
  },
  server: {
    port: 5173,
    proxy: {
      '/ws': { target: 'ws://localhost:8080', ws: true },
      '/api': 'http://localhost:8080',
      '/tiles': 'http://localhost:8080',
    },
  },
});
