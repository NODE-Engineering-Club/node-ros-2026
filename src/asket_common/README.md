# asket_common

Pure-Python geometry, survey planning and heading maths, shared by everything
else in this workspace. No ROS, no third-party dependencies, no I/O.

It exists because three packages have to agree exactly:

* `asket_sim` drives the simulated vessel along the survey lines;
* `gui_backend` draws those lines on the map and paints coverage against them;
* `mission_recorder` records whether the heading was trustworthy per sample.

If `gui_backend` had to import `asket_sim` to get the survey geometry, the
backend would depend on the simulator in production. If each package had its own
copy, the coverage overlay would eventually lie about which parts of the seabed
were actually ensonified — and you would find out back in Windhoek.

## Contents

| Module | What it holds |
|---|---|
| `geo.py` | Local ENU tangent plane, compass-bearing conversions, angle wrapping |
| `survey.py` | Lawnmower pattern, swath half-width, swath ribbon geometry |
| `heading.py` | Heading provenance, validity, heading-vs-COG divergence, seabed error |

## Conventions

* Geographic: WGS84 degrees, altitude in metres.
* Local ENU: east / north / up metres about a per-mission origin.
* Heading: compass degrees, 0 = true north, increasing clockwise. ROS yaw
  conversion happens only at the ROS boundary.

## Testing

```bash
pytest src/asket_common
```

The tests are worth reading before the code — several of them state the physical
claims the rest of the system relies on, such as one degree of heading error
being 87 cm at 50 m range.
