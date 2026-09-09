# Screenshots

`gui-light.png` — the cockpit in mock mode (`npm run dev:mock`), on browser-
generated data with no backend and no vessel. Recording a survey, pre-flight
run, offline tiles on.

Everything in the frame is doing something worth knowing about:

- the **status strip** in the top bar — battery, recording, link — which stays
  put whatever the cockpit is scrolled to;
- the **map**, framed to the whole job: planned lines dashed blue, geofence
  dotted amber, driven track black, and the coverage ribbon in green on one
  side of it, because one sonar covers one side and the gap that leaves is the
  thing this map exists to show;
- the **Advanced** line in the vessel panel, folded away because relay
  positions are debugging detail on a normal day;
- the amber **mounting-geometry warning** in the pre-flight report, which will
  be there on every single run until somebody measures the boat.

The earlier dark-theme captures were deleted rather than kept: the interface is
white now, and a folder of screenshots of a version that no longer exists is
worse than no folder at all.

Regenerating is manual — there is no screenshot test. These document; what the
interface must *do* is pinned by the test suite.
