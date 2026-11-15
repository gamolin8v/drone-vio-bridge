#!/usr/bin/env python3
"""
Camera calibration tool for drone-vio-bridge.

Prints a checkerboard on screen, captures frames from your camera,
computes intrinsics (K, D) and saves config/camera_calibration.yaml.

Usage:
  python calibration/calibrate.py              # auto-detect camera
  python calibration/calibrate.py --type usb --device /dev/video0
  python calibration/calibrate.py --type rpi

Controls during capture:
  SPACE  — capture a frame (need 15+ good frames)
  q      — quit without saving
  s      — compute & save calibration
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np
import yaml

BOARD_W = 9    # inner corners horizontal
BOARD_H = 6    # inner corners vertical
SQUARE  = 0.025  # square size in meters (25 mm)


def _capture_frames(camera_type: str, device: str, width: int, height: int,
                    n_required: int = 15):
    if camera_type == "rpi":
        from picamera2 import Picamera2
        cam = Picamera2()
        cfg = cam.create_still_configuration(main={"size": (width, height)})
        cam.configure(cfg)
        cam.start()
        time.sleep(1)
        def grab():
            arr = cam.capture_array()
            return cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_I420 if arr.ndim==2 else cv2.COLOR_RGB2BGR)
        cleanup = cam.stop
    else:
        idx = int(device.replace("/dev/video", "")) if device.startswith("/dev/video") else int(device)
        cap = cv2.VideoCapture(idx)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        def grab():
            ret, f = cap.read()
            return f if ret else None
        cleanup = cap.release

    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    obj_pts_template = np.zeros((BOARD_H * BOARD_W, 3), np.float32)
    obj_pts_template[:, :2] = np.mgrid[0:BOARD_W, 0:BOARD_H].T.reshape(-1, 2) * SQUARE

    obj_points = []
    img_points = []
    img_size   = None

    print(f"\nCapturing calibration frames ({n_required} required).")
    print("SPACE = capture  |  s = compute+save  |  q = quit\n")

    while True:
        frame = grab()
        if frame is None:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        img_size = gray.shape[::-1]  # (w, h)

        found, corners = cv2.findChessboardCorners(gray, (BOARD_W, BOARD_H), None)
        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, (BOARD_W, BOARD_H), corners, found)
        status = f"Captured: {len(img_points)} / {n_required}"
        cv2.putText(display, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 255, 0) if found else (0, 0, 255), 2)
        cv2.imshow("Calibration — SPACE capture  s save  q quit", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord(' ') and found:
            corners_sub = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            obj_points.append(obj_pts_template)
            img_points.append(corners_sub)
            print(f"  Frame {len(img_points)} captured (found {BOARD_W}×{BOARD_H} corners)")
        if key == ord('s') or len(img_points) >= n_required:
            break

    cleanup()
    cv2.destroyAllWindows()
    return obj_points, img_points, img_size


def _compute_and_save(obj_points, img_points, img_size, out_path: str):
    if len(img_points) < 10:
        print(f"Need at least 10 frames (got {len(img_points)}). Aborting.")
        sys.exit(1)

    print(f"\nComputing calibration from {len(img_points)} frames…")
    flags = cv2.CALIB_RATIONAL_MODEL
    ret, K, D, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, img_size, None, None
    )
    print(f"RMS reprojection error: {ret:.4f} px")
    if ret > 1.5:
        print("WARNING: RMS > 1.5 — capture more frames or use a flatter board")

    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    data = {
        "image_width":  img_size[0],
        "image_height": img_size[1],
        "rms_error":    float(ret),
        "camera_matrix": {
            "rows": 3, "cols": 3,
            "data": K.flatten().tolist(),
        },
        "dist_coeffs": {
            "rows": 1, "cols": len(D.flatten()),
            "data": D.flatten().tolist(),
        },
    }
    with open(out_path, "w") as f:
        yaml.dump(data, f, default_flow_style=False)
    print(f"Calibration saved → {out_path}")
    print(f"\nFocal length: fx={K[0,0]:.1f}  fy={K[1,1]:.1f}")
    print(f"Principal pt: cx={K[0,2]:.1f}  cy={K[1,2]:.1f}")


def main():
    parser = argparse.ArgumentParser(description="Camera calibration for VIO bridge")
    parser.add_argument("--type",   default="usb", choices=["rpi","usb","oak_d"])
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--width",  type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--out",    default="config/camera_calibration.yaml")
    parser.add_argument("--frames", type=int, default=20)
    args = parser.parse_args()

    obj_pts, img_pts, img_sz = _capture_frames(
        args.type, args.device, args.width, args.height, args.frames
    )
    if img_pts:
        _compute_and_save(obj_pts, img_pts, img_sz, args.out)


if __name__ == "__main__":
    main()
