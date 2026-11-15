#!/usr/bin/env python3
"""
drone-vio-bridge — Visual-Inertial Odometry bridge for Raspberry Pi 5.
Feeds position to ArduPilot (MAVLink), iNav (MSP GPS) and Betaflight (MSP RC).

Usage:
  python main.py                       # use config.yaml
  python main.py --config my.yaml
  python main.py --mock                # test without camera/IMU
  python main.py --ardupilot-only      # only ArduPilot bridge
"""

import argparse
import logging
import signal
import sys
import time
import threading
from typing import Optional

# ── Bootstrap path ──────────────────────────────────────────────────────────────
import os
sys.path.insert(0, os.path.dirname(__file__))

from src.config import load_config, AppConfig
from src.vio.base import CameraFrame, IMUSample, Pose
from src.vio.simple_vo import build_vio
from src.sensors.camera import build_camera
from src.sensors.imu import build_imu, IMUSampleBuffer
from src.bridges.mavlink_bridge import build_mavlink_bridge
from src.bridges.msp_bridge import build_msp_bridge
from src.bridges.betaflight_bridge import build_betaflight_bridge


def _setup_logging(level: str):
    fmt = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    logging.basicConfig(level=getattr(logging, level.upper(), logging.INFO),
                        format=fmt, stream=sys.stdout)


log = logging.getLogger("vio-bridge")


class VIOBridgeApp:
    def __init__(self, cfg: AppConfig):
        self._cfg = cfg
        self._running = False

        self._vio = build_vio(cfg)
        self._camera = build_camera(cfg)
        self._imu = build_imu(cfg)
        self._imu_buf = IMUSampleBuffer()

        self._mav   = build_mavlink_bridge(cfg)
        self._msp   = build_msp_bridge(cfg)
        self._bf    = build_betaflight_bridge(cfg)

        self._web: Optional[object] = None
        if cfg.system_web_ui:
            try:
                from src.web.server import WebDashboard
                self._web = WebDashboard(cfg.web.host, cfg.web.port)
            except RuntimeError as e:
                log.warning("Web dashboard unavailable: %s", e)

        self._last_pose: Optional[Pose] = None
        self._frame_times: list = []
        self._pose_lock = threading.Lock()

    # ── Start / Stop ────────────────────────────────────────────────────────

    def start(self):
        log.info("Starting drone-vio-bridge")
        self._running = True

        # Start VIO estimator
        if not self._vio.initialize():
            log.error("VIO initialization failed")
            sys.exit(1)

        # Register IMU callback → buffer
        self._imu.on_sample(self._on_imu)
        self._imu.start()

        # Register camera callback → VIO update
        self._camera.on_frame(self._on_frame)
        self._camera.start()

        # Start output bridges
        if self._mav:
            self._mav.start()
        if self._msp:
            self._msp.start()
        if self._bf:
            self._bf.start()

        # Start web dashboard
        if self._web:
            self._web.start()

        log.info("All components started — waiting for VIO convergence…")

        # Main monitoring loop
        try:
            while self._running:
                self._heartbeat()
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def stop(self):
        log.info("Shutting down…")
        self._running = False
        self._camera.stop()
        self._imu.stop()
        if self._mav:
            self._mav.stop()
        if self._msp:
            self._msp.stop()
        if self._bf:
            self._bf.stop()
        log.info("Shutdown complete")

    # ── Sensor callbacks ────────────────────────────────────────────────────

    def _on_imu(self, ax, ay, az, gx, gy, gz, ts):
        self._imu_buf.push(ax, ay, az, gx, gy, gz, ts)

    def _on_frame(self, image, ts):
        # Drain IMU samples accumulated since last frame
        imu_samples = self._imu_buf.drain()

        # Build CameraFrame (attach latest rangefinder altitude if available)
        frame = CameraFrame(image=image, timestamp=ts)

        # Run VIO update
        pose = self._vio.update(frame, imu_samples)

        if pose is None:
            return

        with self._pose_lock:
            self._last_pose = pose
            self._frame_times.append(ts)
            # Keep only last 2 seconds for FPS estimate
            cutoff = ts - 2.0
            self._frame_times = [t for t in self._frame_times if t > cutoff]

        # Push to all output bridges
        if self._mav:
            self._mav.publish(pose)
        if self._msp:
            self._msp.publish(pose)
        if self._bf:
            self._bf.publish(pose)

        # Update web dashboard
        if self._web:
            fps = len(self._frame_times) / 2.0
            self._web.update(
                pose=pose, fps=fps,
                algorithm=self._cfg.vio.algorithm,
                bridges={
                    "ap": self._mav is not None,
                    "in": self._msp is not None,
                    "bf": self._bf  is not None,
                },
            )

    # ── Heartbeat (1 Hz logging) ────────────────────────────────────────────

    def _heartbeat(self):
        with self._pose_lock:
            pose = self._last_pose
            fps  = len(self._frame_times) / max(1.0,
                       (self._frame_times[-1] - self._frame_times[0])
                       if len(self._frame_times) > 1 else 1.0)

        if pose:
            log.info(
                "pos=(%.2f, %.2f, %.2f) m  rpy=(%.1f, %.1f, %.1f)°  "
                "quality=%.0f%%  fps=%.1f",
                pose.x, pose.y, pose.z,
                pose.roll*57.3, pose.pitch*57.3, pose.yaw*57.3,
                pose.quality*100, fps,
            )
            if self._web:
                self._web.push_log(
                    f"pos=({pose.x:.2f},{pose.y:.2f},{pose.z:.2f})  "
                    f"q={pose.quality*100:.0f}%  fps={fps:.1f}"
                )
        else:
            log.info("VIO not yet converged — waiting for frames…")


# ── Entry point ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="drone-vio-bridge")
    parser.add_argument("--config", default="config.yaml", help="Config file path")
    parser.add_argument("--mock",   action="store_true",
                        help="Force mock camera/IMU/VIO (no hardware needed)")
    parser.add_argument("--ardupilot-only", action="store_true",
                        help="Enable only ArduPilot bridge")
    parser.add_argument("--debug",  action="store_true", help="Verbose logging")
    args = parser.parse_args()

    cfg = load_config(args.config)

    if args.mock:
        cfg.camera.type = "mock"
        cfg.imu.type    = "mock"
        cfg.vio.algorithm = "mock"

    if args.ardupilot_only:
        cfg.output.inav.enabled = False
        cfg.output.betaflight.enabled = False

    if args.debug:
        cfg.system_log_level = "DEBUG"

    _setup_logging(cfg.system_log_level)

    app = VIOBridgeApp(cfg)

    # Graceful SIGTERM shutdown
    def _sigterm(sig, frame):
        log.info("SIGTERM received")
        app.stop()
        sys.exit(0)

    signal.signal(signal.SIGTERM, _sigterm)

    app.start()


if __name__ == "__main__":
    main()
