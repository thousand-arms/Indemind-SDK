"""Python wrapper for the INDEMIND IMSEE-SDK (stereo + IMU)."""

from __future__ import annotations

# The extension module bakes DT_RPATH entries (see setup.py) that point at
# the SDK's lib/ dirs and this package's opencv_compat/. Because setup.py
# links with --disable-new-dtags, the RPATH is transitive: when the linker
# loads the extension, it uses these paths not only for libindemind.so but
# for libindemind's own NEEDED entries (libopencv_*.so.3.4, libMNN.so).
# opencv_compat/ ships symlinks from the 3.4 sonames to the system OpenCV
# 4 libs, so a single OpenCV ends up loaded and cv2 can coexist in the
# same process.

from ._pyindemind_ext import Camera, ImuSample

__all__ = ["Camera", "ImuSample"]
