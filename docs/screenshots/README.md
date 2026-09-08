# Screenshots

Captured from `npm run dev:mock` — the GUI running on browser-generated data,
no backend and no vessel. Chromium at 1800x1050.

| | Shows |
|---|---|
| `01-overview.png` | The whole cockpit: map, panels, and the mock control column |
| `02-topbar.png` | Receiving chip, confirmed mode badge, alarm count |
| `03-mock-controls.png` | The dev panel — every fault that can be injected |
| `10-alarms.png` … `19-link.png` | Each panel on its own, nominal |
| `20-preflight-report.png` | A GO/NO-GO report, including the amber mounting-geometry warning |
| `21-mode-command-failed.png` | A mode command with no confirmation: **"the vessel has NOT changed state"** |

Regenerating them is manual — there is no screenshot test. The point of these
is documentation, not verification; what the interface must do is pinned by the
test suite instead.
