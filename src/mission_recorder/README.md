# mission_recorder

Writes the mission files, lists what is on the disk, and exports to a fast path
with verification.

## The problem this package exists to solve

**The sonar knows nothing about where it is.** It records angles and times.
Position and heading come from the vessel and must be recorded *alongside* the
sonar stream so the two can be merged afterwards.

The chosen approach (Option B, decided with the team) is to record the raw sonar
stream and a timestamped trajectory as two separate files and pair them
afterwards on `utc_msec`. SonarView does not have to run during the mission.

Two things follow, and they are the whole design:

* **Clock discipline is everything.** If the sonar's clock drifts from the
  Jetson's, the survey is not degraded — it is worthless, and nobody finds out
  until the data is opened back home. The recorder logs `clock_offset_ms` with
  every diagnostics snapshot, so if it does happen the damage is at least
  visible in the recording.
* **The trajectory must bracket the sonar stream.** A record is written at
  mission start, before any sonar byte, and again at stop. Without them the
  first and last pings fall outside the trajectory and cannot be placed at all.

## Layout

```
/data/missions/<name>_<UTC timestamp>/
    manifest.json          metadata, versions, config snapshot
    sonar_raw.bin          raw Ping Protocol stream, unmodified
    trajectory.jsonl       one record per sample, ~10 Hz
    diagnostics.jsonl      health snapshots, including the clock offset
    events.jsonl           mode changes, alarms, operator actions
    rosbag/                full rosbag2 (optional, config flag)
    live.svlog             SonarView-readable log (optional, config flag)
    checksums.sha256
```

`manifest.json` is written at start **and** updated at stop. A mission that ends
because the battery died still leaves a readable, self-describing directory, and
it is still listed — marked incomplete. Hiding it would hide exactly the mission
most likely to need attention.

`trajectory.jsonl` records `heading_source` and `heading_valid` **per sample**.
Those are not decoration: the trajectory has to record not just the heading but
how trustworthy it was at that instant, so a window of compromised data can be
identified afterwards rather than guessed at.

`sonar_raw.bin` is the stream byte for byte as it arrived — published by
`omniscan_bridge` on `/sonar/raw` *before* parsing, so a frame the parser
rejects still reaches the recording. Anything reinterpreted before it reaches
disk is something a post-mission tool cannot reinterpret differently later.

## Disk

The recorder stops cleanly rather than filling the disk. A full file system
takes the whole Jetson down, not just the recording, and a mission that stops
with a valid manifest and checksums is salvageable. It writes a `disk_full`
event first, so the reason is in the record.

## Export

Full mission data is 2–5 GB per hour. **It never goes over the wireless link.**

* **Blocked while recording** — copying gigabytes competes with the recorder for
  CPU and disk, and losing survey data to save time on a transfer is a bad trade.
* **Verified** — SHA-256 on the *copy*, so the original can be deleted safely.
  A failed verification says so and says not to delete the original.
* **Explained when unavailable** — *"Transfer unavailable — connect a USB drive
  or an Ethernet cable"*, not a dead button. The Jetson's own storage is never
  offered as a destination: copying a mission onto the disk it already lives on
  fills that disk and stops the next recording.

## Testing in sim

```bash
pytest src/mission_recorder      # no ROS, no hardware
```

A whole simulated mission, end to end, including the post-mission pairing:

```bash
python3 -m gui_backend.core.app --sim --port 8080
# then in the GUI: Recording -> name -> Start
```

The mission directory a simulated run produces is the one a real run produces:
the simulator's Ping Protocol frames are the same bytes, so the fusion path can
be rehearsed before anyone flies to Namibia.
