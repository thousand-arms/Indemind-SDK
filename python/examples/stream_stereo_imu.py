"""Open the INDEMIND camera, show stereo frames with cv2, print IMU.

Usage:
    python3 examples/stream_stereo_imu.py

Press 'q' or ESC in either window to quit.
"""

import time

import cv2
import numpy as np

import pyindemind


def main():
    cam = pyindemind.Camera()
    if not cam.start(resolution="640x400", img_hz=50, imu_hz=1000):
        raise SystemExit("Failed to start INDEMIND camera. "
                         "Check USB permissions (udev rule) and that the "
                         "device is plugged in.")

    try:
        params = cam.get_module_params()
        print(f"Device: {params['device']['id']}  "
              f"baseline={params['baseline_m']:.4f} m  "
              f"IMU chip={params['device']['imu_chip']}")

        cv2.namedWindow("INDEMIND Left", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("INDEMIND Right", cv2.WINDOW_AUTOSIZE)

        imu_count = 0
        last_fps_t = time.monotonic()
        frames_since = 0

        while True:
            frame = cam.get_frame(timeout_s=0.1)
            if frame is not None:
                ts, left, right = frame
                cv2.imshow("INDEMIND Left", left)
                cv2.imshow("INDEMIND Right", right)

                frames_since += 1
                now = time.monotonic()
                if now - last_fps_t >= 2.0:
                    fps = frames_since / (now - last_fps_t)
                    print(f"[IMG] t={ts:.4f}s  shape={left.shape}  {fps:.1f} fps")
                    last_fps_t = now
                    frames_since = 0

            for s in cam.drain_imu():
                if imu_count % 200 == 0:
                    a = s.accel
                    g = s.gyro
                    print(f"[IMU] t={s.timestamp:.4f}s  "
                          f"accel=({a[0]:+.3f}, {a[1]:+.3f}, {a[2]:+.3f}) m/s^2  "
                          f"gyro=({g[0]:+.3f}, {g[1]:+.3f}, {g[2]:+.3f}) rad/s")
                imu_count += 1

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break

    finally:
        cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
