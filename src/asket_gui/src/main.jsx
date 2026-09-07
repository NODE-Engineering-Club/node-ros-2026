import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';

import { App } from './App.jsx';
import { Connection } from './lib/connection.js';
import './styles.css';

const connection = new Connection();
connection.connect();

createRoot(document.getElementById('root')).render(
  <StrictMode>
    <App connection={connection} />
  </StrictMode>,
);
