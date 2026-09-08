# Field setup

Steps that must be done on the hardware and cannot be done from this
repository. Each one, if missed, produces a survey that looks fine on the day
and is unusable afterwards.

## 1. The sonar's clock (do this first)

**This is the single most expensive step to get wrong.** Post-mission fusion
pairs the sonar stream and the trajectory on the sonar's own `utc_msec`. If
that clock is wrong, the survey is not degraded — it is worthless, and nobody
finds out until the data is opened back home.

Cerulean **removed** the Omniscan 3D's `SET_NTP_INFO` packet. The NTP server is
configured on the device, not by us, and **the default is an internet host**
(`time.google.com`). There is no internet in the field.

So:

1. Run an NTP server on the Jetson, disciplined by GPS time. There is no
   internet, so it must be a local, GPS-backed source.
2. Point the sonar at it — on the device, through its configuration file or
   SonarLink. Not from `omniscan_bridge`; there is no packet for it.
3. Confirm before every deployment. `omniscan_bridge` publishes
   `clock_offset_ms` in `/sonar/status`, the GUI's sonar panel shows it, and
   the pre-flight check fails at 250 ms with *"Everything recorded will be
   un-georeferenceable"*.

The software can measure this and shout about it. It cannot fix it.

## 2. USB drives on a headless Jetson

A headless Jetson has no desktop session, so **nothing auto-mounts a USB
drive** and `/media/*` stays empty. Export would simply never offer a
destination.

Install the rule and unit shipped in `deploy/`:

```bash
sudo cp deploy/99-asket-usb.rules /etc/udev/rules.d/
sudo cp deploy/media-asket.mount deploy/media-asket.automount /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now media-asket.automount
sudo udevadm control --reload-rules
```

Then check it:

```bash
python3 -m mission_recorder.check_export_paths
```

That prints what the recorder can actually see, and says why anything is
rejected. It tests the **mount**, not the directory: a directory that exists
but is not a mount point is the Jetson's own eMMC with a folder on it, and
copying a mission there fills the disk the missions already live on and stops
the next recording.

## 3. Mounting geometry

The sonar's mounting angle and the lever arm from the GNSS antenna to the
transducer must be **measured to the centimetre** and put in
`src/omniscan_bridge/config/mounting.yaml`. It is a systematic offset that no
amount of post-processing will discover.

Until they are measured, the file stays marked `PROVISIONAL` and the pre-flight
check returns a **WARN** naming the file. That is deliberate: deploying on
default geometry by accident is exactly the mistake worth an extra amber line.

## 4. Offline map tiles

There is no internet in the field. Pre-download an `.mbtiles` covering the
survey box and put it where `gui_backend` expects it
(`tiles_path`, default `/data/maps/survey.mbtiles`).

Without it the map degrades to a labelled coordinate grid and says so — usable,
but you will not see the coastline. Check it before leaving:

```bash
curl -s http://<jetson>:8080/api/tiles/info
```

## 5. After the mission — produce the .svlog

SonarView cannot import an external trajectory. Merge the recorded streams into
a log it can open:

```bash
python3 -m mission_recorder.merge_svlog /data/missions/<mission>
python3 -m mission_recorder.merge_svlog --all /data/missions
```

Read the summary it prints. It warns when pings fell outside the trajectory and
have no position, and when a stretch was recorded with an invalid heading.
