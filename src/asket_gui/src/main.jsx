import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';

import { App } from './App.jsx';
import { Connection } from './lib/connection.js';
import './styles.css';

/**
 * Mock mode is chosen at build/dev time, not at runtime.
 *
 *     VITE_MOCK=true npm run dev        # or: npm run dev:mock
 *
 * The two modes share every component. The only difference is which socket the
 * connection opens — a real WebSocket, or one that generates the data in this
 * browser. Anything else would mean reviewing a GUI that is not the GUI.
 */
const useMock = import.meta.env.VITE_MOCK === 'true' || import.meta.env.VITE_MOCK === '1';

async function boot() {
  let connection;
  if (useMock) {
    const { createMockConnection } = await import('./lib/mock/index.js');
    connection = createMockConnection({ timeScale: 4 });
  } else {
    connection = new Connection();
  }
  connection.connect();

  createRoot(document.getElementById('root')).render(
    <StrictMode>
      <App connection={connection} />
    </StrictMode>,
  );
}

boot();
