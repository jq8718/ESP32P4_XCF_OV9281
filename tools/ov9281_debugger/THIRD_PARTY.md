# Third-party components

The desktop executable bundles dynamically loaded runtime libraries. Source code for this application,
its pinned requirements and its PyInstaller build script are provided in `tools/ov9281_debugger`.
The environment can be rebuilt with compatible replacement runtime libraries.

| Component | Version | Upstream license / source |
| --- | --- | --- |
| PySide6-Essentials / Shiboken6 / Qt | 6.8.3 | LGPL-3.0 option; https://code.qt.io/cgit/pyside/pyside-setup.git/ and https://code.qt.io/cgit/qt/ |
| opencv-python-headless | 4.12.0.88 | Wrapper MIT, OpenCV Apache-2.0; https://github.com/opencv/opencv-python and https://github.com/opencv/opencv |
| NumPy | Installed dependency version recorded by pip | BSD-3-Clause, with bundled library notices; https://github.com/numpy/numpy |
| pyserial | 3.5 | BSD-3-Clause; https://github.com/pyserial/pyserial |
| CPython | 3.11.2 | PSF license; https://www.python.org/downloads/release/python-3112/ |
| PyInstaller | 6.16.0 | GPL-2.0-or-later with bootloader exception; https://github.com/pyinstaller/pyinstaller |

LGPL and GPL license text: https://www.gnu.org/licenses/lgpl-3.0.html and https://www.gnu.org/licenses/gpl-3.0.html .
