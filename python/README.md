# pyindemind

Minimal Python bindings for the INDEMIND IMSEE-SDK. Exposes raw stereo
images, IMU samples, and module calibration — nothing else (no SLAM,
depth, disparity, or detector).

## Requirements

- Linux x86_64 (Ubuntu 22.04 tested)
- Python ≥ 3.8
- System OpenCV 4 dev headers for the build: `sudo apt install libopencv-dev`
- **OpenCV 3.4 runtime libraries** — see below.
- The INDEMIND udev rule installed (see the ORB-SLAM3 calibration notes
  at `/home/andrew/Dev/ORB_SLAM3/Examples/Calibration/indemind_instructions.md`).

### OpenCV 3.4 runtime libraries

`libindemind.so` was built against OpenCV 3.4 and hard-depends on
`libopencv_{core,imgproc,calib3d,videoio}.so.3.4`. OpenCV 4 is *not*
ABI-compatible for every symbol `libindemind` uses (e.g. `cv::rectangle`).

Drop the four `libopencv_*.so.3.4*` files into
`pyindemind/opencv_compat/`. Easiest: build a minimal OpenCV 3.4 once,
following the instructions in `indemind_instructions.md`, then copy from
`~/Dev/opencv_3.4/lib/`:

```bash
for L in core imgproc calib3d videoio; do
  cp -P ~/Dev/opencv_3.4/lib/libopencv_${L}.so.3.4* \
        pyindemind/opencv_compat/
done
```

These libraries live alongside the extension and are found at runtime via
`DT_RPATH` baked by `setup.py`. They do **not** leak into the `cv2`
(OpenCV 4) that your Python code imports — both versions coexist in the
same process because they have distinct sonames.

## Install

```bash
cd python
pip install -e .
```

The extension bakes `$ORIGIN`-relative and absolute `DT_RPATH` entries
that assume the editable layout (`python/pyindemind/` next to the repo's
`lib/` and `src/detector/lib/`). For a non-editable install, edit the
RPATHs in `setup.py` or copy `libindemind.so` + `libMNN.so` alongside
`opencv_compat/`.

## Usage

```python
import pyindemind

cam = pyindemind.Camera()
cam.start(resolution="640x400", img_hz=50, imu_hz=1000)

# Stereo frames — numpy uint8, HxW (single-channel) or HxWxC
result = cam.get_frame(timeout_s=0.1)
if result is not None:
    ts, left, right = result
    print(ts, left.shape, right.shape)

# IMU — drained in batches. Default units are SI (m/s², rad/s).
# Pass si_units=False to cam.start() for raw device units (g, deg/s).
for s in cam.drain_imu():
    print(s.timestamp, s.accel, s.gyro)

# Calibration / device info
params = cam.get_module_params()
print(params["imu"]["sigma_g_c"], params["baseline_m"])

cam.stop()
```

`pyindemind` and `cv2` can be imported in either order in the same
process — verified with `cv2` 4.12 against a real OpenCV 3.4.20 build.

## Notes on units

The device outputs accelerometer in g and gyro in deg/s. With
`si_units=True` (default) the wrapper converts to m/s² and rad/s to match
EuRoC/Kalibr conventions. The conversion happens before data lands in the
queue; raw bytes from the USB stack are never visible to callers.
