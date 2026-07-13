"""LiDAR-camera extrinsic calibration solver.

Solves for the 6-DOF rigid transform from the LiDAR frame to the camera
frame using manually collected 3D-2D point correspondences and OpenCV's
Perspective-n-Point (PnP) algorithm with Levenberg-Marquardt refinement.

Method adapted from:
  [1] L. Zhang et al., "Calibration Method of 2D LIDAR and Camera Based on
      Indoor Structural Features," Hohai University.
      https://www.researching.cn/articles/OJbfdef44a334f8d3f
  [2] Q. Zhang and R. Pless, "Extrinsic calibration of a camera and laser
      range finder (improves camera calibration)," IROS 2004, pp. 2301-2306.
  [3] X. Zhong, camera_lidar_calibration, GitHub, 2018.
      https://github.com/TurtleZhong/camera_lidar_calibration

Reads ~/.ros/lidar_camera_data.txt (x y z u v per line),
solves for the 6-DOF transform from lidar frame to camera frame using
cv2.solvePnP + LM refinement, and writes the result to
~/.ros/lidar_camera_extrinsic.yaml.

Usage:
    ros2 run calibration calibrate
    # or directly:
    python3 -m calibration.calibrate
"""

import math
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml


DATA_PATH      = Path.home() / ".ros" / "lidar_camera_data.txt"
EXTRINSIC_PATH = Path.home() / ".ros" / "lidar_camera_extrinsic.yaml"
CAMERA_INFO    = Path(__file__).parents[3] / "src/bringup/config/front_camera.yaml"


def _load_camera_info(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with path.open() as f:
        info = yaml.safe_load(f)
    K = np.array(info["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    D = np.array(info["distortion_coefficients"]["data"], dtype=np.float64)
    return K, D


def _load_data(path: Path) -> tuple[np.ndarray, np.ndarray]:
    pts3, pts2 = [], []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        x, y, z, u, v = map(float, line.split())
        pts3.append([x, y, z])
        pts2.append([u, v])
    return np.array(pts3, dtype=np.float64), np.array(pts2, dtype=np.float64)


def _rvec_to_rpy(rvec: np.ndarray) -> tuple[float, float, float]:
    R, _ = cv2.Rodrigues(rvec)
    # Roll (x), Pitch (y), Yaw (z) from rotation matrix
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        roll  = math.atan2( R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw   = math.atan2( R[1, 0], R[0, 0])
    else:
        roll  = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw   = 0.0
    return roll, pitch, yaw


def main(args=None):
    # ── Load inputs ───────────────────────────────────────────────────────────
    if not DATA_PATH.exists():
        print(f"ERROR: data file not found: {DATA_PATH}", file=sys.stderr)
        print("Run:  ros2 launch bringup calibrate_lidar_camera.launch.py", file=sys.stderr)
        sys.exit(1)

    pts3d, pts2d = _load_data(DATA_PATH)
    n = len(pts3d)
    print(f"Loaded {n} point pairs from {DATA_PATH}")
    if n < 6:
        print(f"ERROR: need ≥ 6 pairs (have {n})", file=sys.stderr)
        sys.exit(1)

    # Try workspace path first, fall back to ~/.ros/front_camera.yaml
    cam_path = CAMERA_INFO if CAMERA_INFO.exists() else Path.home() / ".ros" / "front_camera.yaml"
    if not cam_path.exists():
        print(f"ERROR: camera_info YAML not found at {cam_path}", file=sys.stderr)
        sys.exit(1)
    K, D = _load_camera_info(cam_path)
    print(f"Camera intrinsics loaded from {cam_path}")

    # ── Solve PnP ─────────────────────────────────────────────────────────────
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        pts3d, pts2d, K, D,
        flags=cv2.SOLVEPNP_ITERATIVE,
        reprojectionError=8.0,
        confidence=0.999,
    )
    if not ok:
        print("ERROR: solvePnPRansac failed — collect more / better pairs.", file=sys.stderr)
        sys.exit(1)

    n_inliers = inliers.shape[0] if inliers is not None else n
    print(f"RANSAC inliers: {n_inliers}/{n}")

    # Refine on inliers only
    if inliers is not None:
        pts3_in = pts3d[inliers.flatten()]
        pts2_in = pts2d[inliers.flatten()]
    else:
        pts3_in, pts2_in = pts3d, pts2d

    _, rvec, tvec = cv2.solvePnP(pts3_in, pts2_in, K, D, rvec, tvec,
                                  useExtrinsicGuess=True,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    cv2.solvePnPRefineLM(pts3_in, pts2_in, K, D, rvec, tvec)

    # ── Reprojection error ────────────────────────────────────────────────────
    proj, _ = cv2.projectPoints(pts3_in, rvec, tvec, K, D)
    errors  = np.linalg.norm(pts2_in - proj.squeeze(1), axis=1)
    mean_err = errors.mean()
    max_err  = errors.max()
    print(f"Reprojection error (inliers): mean={mean_err:.2f} px  max={max_err:.2f} px")
    if mean_err > 10.0:
        print("WARNING: error > 10 px — consider collecting more pairs or rechecking clicks.")

    # ── Convert to (x,y,z, roll,pitch,yaw) ───────────────────────────────────
    tx, ty, tz = tvec.flatten()
    roll, pitch, yaw = _rvec_to_rpy(rvec)

    print(f"\nLiDAR → Camera extrinsic:")
    print(f"  translation : x={tx:.4f}  y={ty:.4f}  z={tz:.4f}  [m]")
    print(f"  rotation    : roll={math.degrees(roll):.2f}°  pitch={math.degrees(pitch):.2f}°  yaw={math.degrees(yaw):.2f}°")

    # ── Save ──────────────────────────────────────────────────────────────────
    result = {
        "lidar_to_camera": {
            "x":     float(tx),
            "y":     float(ty),
            "z":     float(tz),
            "roll":  float(roll),
            "pitch": float(pitch),
            "yaw":   float(yaw),
        },
        "reprojection_error_px": float(mean_err),
        "n_pairs":    n,
        "n_inliers":  int(n_inliers),
    }
    EXTRINSIC_PATH.parent.mkdir(parents=True, exist_ok=True)
    with EXTRINSIC_PATH.open("w") as f:
        yaml.safe_dump(result, f, default_flow_style=False)
    print(f"\nSaved → {EXTRINSIC_PATH}")
    print("Next steps:")
    print(f"  cp {EXTRINSIC_PATH} src/bringup/config/lidar_camera_extrinsic.yaml")


if __name__ == "__main__":
    main()
