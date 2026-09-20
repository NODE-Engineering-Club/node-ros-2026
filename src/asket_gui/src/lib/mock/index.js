// Mock mode entry point.
//
// Builds a `Connection` whose socket is a `MockTransport` instead of a
// WebSocket. Everything above the socket is the real client, so what you are
// reviewing in mock mode is the actual interface — not a second implementation
// that happens to look like it.

import { Connection } from '../connection.js';
import { MockTransport } from './mockTransport.js';
import { MockWorld } from './world.js';

export { MockWorld } from './world.js';
export { SCENARIOS } from './scenarios.js';

/** Synthetic raster tiles, drawn in the browser. */
export const MOCK_TILE_PROTOCOL = 'mocktiles';

export function createMockConnection(options = {}) {
  const world = new MockWorld(options.world);
  let transport = null;

  const connection = new Connection('mock://asket', {
    isMock: true,
    transport: () => {
      transport = new MockTransport(world, { timeScale: options.timeScale ?? 1 });
      return transport;
    },
    // No backend to ask, so the answer is supplied. `tilesAvailable` is
    // switchable from the dev panel, because "the map degrades honestly with no
    // tiles" is a behaviour worth being able to see rather than take on trust.
    tileInfo: () =>
      Promise.resolve(
        world.mockTilesAvailable
          ? {
              available: true,
              name: 'Synthetic test tiles',
              format: 'png',
              min_zoom: 0,
              max_zoom: 22,
              bounds: null,
              tile_count: -1,
              message:
                'Synthetic tiles generated in the browser. Not a real chart — they ' +
                'exist so the tiled path can be reviewed without an .mbtiles file.',
            }
          : {
              available: false,
              message:
                'No offline tile file. The map shows a coordinate grid only. In the ' +
                'field this is what you get if the .mbtiles for the survey area is ' +
                'missing from the Jetson.',
            },
      ),
  });

  // The dev panel drives the world directly; the transport is exposed so it can
  // force a profile without going through the operator-facing controls.
  connection.mock = {
    world,
    get transport() {
      return transport;
    },
  };
  return connection;
}
