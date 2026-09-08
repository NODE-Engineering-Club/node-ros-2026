// Synthetic offline tiles, drawn in the browser.
//
// The requirement is that the map works with no internet. In the field that is
// met by a pre-downloaded .mbtiles file on the Jetson; in mock mode there is no
// Jetson and no file, so tiles are generated on demand through a MapLibre
// custom protocol. Nothing is fetched and nothing is bundled.
//
// These are obviously not a chart — they are a grid with coordinates and a
// water-coloured ground. That is deliberate: the point is to review the *tiled*
// rendering path (that layers sit correctly over a raster basemap) without
// anybody mistaking the result for real bathymetry.

const TILE_SIZE = 256;

/** Register `mocktiles://` with a MapLibre instance. */
export function registerMockTileProtocol(maplibregl, protocol = 'mocktiles') {
  if (maplibregl.__asketMockTilesRegistered) return;
  maplibregl.__asketMockTilesRegistered = true;

  maplibregl.addProtocol(protocol, async (params) => {
    // mocktiles://{z}/{x}/{y}
    const [z, x, y] = params.url
      .replace(`${protocol}://`, '')
      .split('/')
      .map((part) => parseInt(part, 10));
    const blob = await drawTile(z, x, y);
    return { data: await blob.arrayBuffer() };
  });
}

function drawTile(z, x, y) {
  const canvas = document.createElement('canvas');
  canvas.width = TILE_SIZE;
  canvas.height = TILE_SIZE;
  const ctx = canvas.getContext('2d');

  // A sea-ish ground that shades with latitude, so panning is visibly moving.
  const shade = 24 + ((x * 7 + y * 13) % 10);
  ctx.fillStyle = `rgb(${shade - 6}, ${shade + 6}, ${shade + 22})`;
  ctx.fillRect(0, 0, TILE_SIZE, TILE_SIZE);

  ctx.strokeStyle = 'rgba(140, 170, 200, 0.25)';
  ctx.lineWidth = 1;
  for (let i = 64; i < TILE_SIZE; i += 64) {
    ctx.beginPath();
    ctx.moveTo(i, 0);
    ctx.lineTo(i, TILE_SIZE);
    ctx.moveTo(0, i);
    ctx.lineTo(TILE_SIZE, i);
    ctx.stroke();
  }

  ctx.strokeStyle = 'rgba(140, 170, 200, 0.5)';
  ctx.strokeRect(0.5, 0.5, TILE_SIZE - 1, TILE_SIZE - 1);

  ctx.fillStyle = 'rgba(160, 190, 220, 0.7)';
  ctx.font = '11px ui-monospace, monospace';
  ctx.fillText(`${z}/${x}/${y}`, 8, 18);
  ctx.fillText(`${tileNorth(z, y).toFixed(4)}°`, 8, 34);
  ctx.fillText(`${tileWest(z, x).toFixed(4)}°`, 8, 48);
  ctx.fillText('SYNTHETIC — not a chart', 8, TILE_SIZE - 10);

  return new Promise((resolve) => canvas.toBlob(resolve, 'image/png'));
}

function tileWest(z, x) {
  return (x / 2 ** z) * 360 - 180;
}

function tileNorth(z, y) {
  const n = Math.PI - (2 * Math.PI * y) / 2 ** z;
  return (180 / Math.PI) * Math.atan(0.5 * (Math.exp(n) - Math.exp(-n)));
}
