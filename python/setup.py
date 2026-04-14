import platform
import shlex
import subprocess
from pathlib import Path

from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup

SDK_ROOT = Path(__file__).resolve().parent.parent

if platform.system() != "Linux" or platform.machine() != "x86_64":
    raise RuntimeError(
        f"pyindemind currently only builds for Linux x86_64 "
        f"(got {platform.system()} {platform.machine()}). "
        f"Extend setup.py to select the matching lib/ variant for your arch."
    )

INDEMIND_LIB_DIR = SDK_ROOT / "lib" / "others" / "x64-opencv3.4.3"
MNN_LIB_DIR = SDK_ROOT / "src" / "detector" / "lib" / "x86-64"

for p, what in [(INDEMIND_LIB_DIR / "libindemind.so", "libindemind.so"),
                (MNN_LIB_DIR / "libMNN.so", "libMNN.so")]:
    if not p.exists():
        raise RuntimeError(f"Missing {what} at {p}")


def _pkgconfig_includes(pkg):
    out = subprocess.check_output(["pkg-config", "--cflags-only-I", pkg]).decode()
    return [tok[2:] for tok in shlex.split(out) if tok.startswith("-I")]


try:
    cv_includes = _pkgconfig_includes("opencv4")
except (subprocess.CalledProcessError, FileNotFoundError):
    try:
        cv_includes = _pkgconfig_includes("opencv")
    except (subprocess.CalledProcessError, FileNotFoundError):
        raise RuntimeError(
            "OpenCV development headers not found. "
            "Install with: sudo apt install libopencv-dev"
        )

# DT_RPATH (not DT_RUNPATH) so libindemind.so's own NEEDED entries
# for libopencv_*.so.3.4 + libMNN.so resolve transitively at runtime.
# $ORIGIN points at the installed extension's directory.
# Paths assume editable install (`pip install -e .`) so the extension lives
# at `python/pyindemind/` relative to the SDK lib dirs.
rpaths = [
    "$ORIGIN/opencv_compat",
    str(INDEMIND_LIB_DIR),
    str(MNN_LIB_DIR),
]

ext = Pybind11Extension(
    "pyindemind._pyindemind_ext",
    ["src/bindings.cpp"],
    include_dirs=[str(SDK_ROOT / "include")] + cv_includes,
    library_dirs=[str(INDEMIND_LIB_DIR), str(MNN_LIB_DIR)],
    libraries=["indemind", "MNN", "pthread"],
    extra_link_args=[
        "-Wl,--allow-shlib-undefined",
        "-Wl,--disable-new-dtags",
        *[f"-Wl,-rpath,{p}" for p in rpaths],
    ],
    cxx_std=14,
)

setup(ext_modules=[ext], cmdclass={"build_ext": build_ext})
