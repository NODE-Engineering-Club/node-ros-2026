# deploy

Host configuration for the Jetson. None of this can be done from inside the
ROS workspace, and all of it is documented in [`../docs/SETUP.md`](../docs/SETUP.md).

## USB mounting

A headless Jetson has no desktop session, so **nothing auto-mounts a USB
drive**. `/media/*` stays empty, and mission export never offers a destination.
On a beach that looks like a software bug.

Two ways to fix it. Pick one — they overlap.

| | When to use it | Mount point |
|---|---|---|
| `media-asket.mount` + `media-asket.automount` | **Preferred.** One dedicated drive, labelled `ASKET` | always `/media/asket` |
| `99-asket-usb.rules` | Any stick, whatever it is called | `/media/asket/<label or device>` |

The labelled pair is deterministic: the same path every time, whichever port
the drive is in, and one fewer thing to think about at 06:00 on a beach. The
udev rule is the general fallback for when somebody hands you a random stick.

Format the drive **ext4**. Mission files exceed FAT32's 4 GB single-file limit
within about ninety minutes of survey.

## Verifying

```bash
python3 -m mission_recorder.check_export_paths
```

That prints what the recorder can actually see and, for anything it rejects,
why. It tests the **mount**, not the directory — a directory that exists but is
not a mount point is a folder on the Jetson's own disk, and copying a mission
there fills the disk the missions live on and stops the next recording.
