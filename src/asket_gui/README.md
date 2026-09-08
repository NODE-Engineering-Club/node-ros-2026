# asket_gui

The mission cockpit. React + MapLibre GL, built with Vite, served by
`gui_backend`.

```bash
npm install
npm run build     # output goes straight into gui_backend/gui_backend/static/
npm run dev       # dev server on :5173, proxying to the backend on :8080
```

The dependency tree is deliberately small — React, MapLibre and Vite, nothing
else. It has to build offline on arm64, and every kilobyte of it crosses the
shore link on first load.

## Layout priority

Map first, vessel state always visible, everything else secondary. Designed for
a laptop screen outdoors in bright sunlight: high contrast, large type for the
values that matter, and no low-contrast greys used to carry meaning.

## Where the safety rules live

| Rule | Where |
|---|---|
| 3 — the soft ESTOP is "Cut propulsion", never "Emergency stop" | `src/lib/labels.js` owns every command string, so the wording is one decision in one place |
| 4 — displayed state is confirmed state | `panels/VesselState.jsx` renders only what the Pico reports. `panels/ModeCommands.jsx` renders the *command*, separately, and never touches the displayed mode |
| 5 — two-step confirmation | `components/ConfirmButton.jsx`. Arm, then confirm; the armed state times out on its own |
| 7 — data age is always visible | `components/Value.jsx` and `lib/staleness.js`. Stale values are degraded loudly — struck through and red — because nobody notices a slightly different grey in sunlight |

## Offline maps

There is no internet in the field, so no style, glyph or sprite is fetched from
a URL. Raster tiles come from the `.mbtiles` file on the Jetson via
`/tiles/{z}/{x}/{y}.png`.

If that file is missing, or the vessel is outside the area it covers, the map
draws a **coordinate graticule** and says so in plain words. It never silently
shows an empty rectangle — an operator would read that as "no map today"
rather than "fix the tile file".

The vessel's heading arrow stops rotating and turns red when heading is
invalid. Pointing an arrow confidently in a direction we do not trust is exactly
the lie this GUI must not tell.

## Coverage

The coverage overlay is built as a **ribbon of quads between consecutive
samples**, not one filled polygon. A gap in the data therefore renders as a gap
in the ribbon. With one sonar unit covering one side only, a gap that renders as
filled is the single most expensive way this GUI could mislead an operator —
you find out back in Windhoek.

## Bandwidth

`track` and `coverage` are **append-only** streams: the server sends only what
is new since this client last heard, and the client splices it on. Sending the
whole history each frame reached 18 kB per frame after ninety seconds and would
have been about a megabyte per frame after three hours — the exact
"works on the bench, collapses offshore" failure the architecture is meant to
prevent. Measured against the simulator, the full subscription set costs about
6–8 kB/s and stays flat as the mission runs.

If an increment arrives that cannot be spliced onto what we hold — a dropped
frame, or a reconnect — the stream is reset and a full resync requested. A
visibly short history is recoverable; a silently wrong coverage ribbon is not.

The one thing that does grow with mission length is the **initial** resync a
newly connected client receives. At `reduced` detail a three-hour survey is
roughly 160 kB, which is about a second and a half of 4G.

## Data age

Every live value carries its age. Age is computed against an estimate of the
*server's* clock rather than the laptop's, so a laptop with a wrong clock
cannot make everything look permanently fresh. See `lib/connection.js`.

Staleness thresholds scale with the negotiated rate: a 0.2 Hz stream on the
beacon profile is not stale at four seconds old, and painting the screen red
because the link is working exactly as negotiated would teach the operator to
ignore the colour.

The header reports what has actually been **received**, not what the socket
believes about itself. A TCP connection stays open for tens of seconds after
the link behind it has gone, and for all of that time `connected` is true while
nothing is arriving.
