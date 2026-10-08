# Build with tools/build_gui.py so verified vendor files and notices are staged.
import os
from pathlib import Path

repo = Path(SPECPATH).parent
assets = repo / "build/gui-assets"
onedir = os.environ.get("STM32FC_GUI_ONEDIR") == "1"
a = Analysis(
    [str(repo / "tools/gui.py")],
    pathex=[str(repo / "tools")],
    binaries=[(str(assets / "vendor/dfu-util" / name), "vendor/dfu-util")
              for name in ("dfu-util.exe", "libusb-1.0.dll")],
    datas=[(str(assets / "licenses"), "licenses"),
           (str(assets / "build-info.json"), ".")] +
          [(str(p), "sources") for p in (assets / "sources").iterdir()
           if p.is_file() and "everywhere-src" not in p.name],
    hiddenimports=[], hookspath=[], hooksconfig={}, runtime_hooks=[],
    excludes=[], noarchive=False,
)
pyz = PYZ(a.pure)
exe = EXE(
    pyz, a.scripts, *([] if onedir else [a.binaries, a.datas]), [],
    exclude_binaries=onedir, name="STM32FC-GUI", debug=False,
    bootloader_ignore_signals=False, strip=False, upx=False,
    console=False, disable_windowed_traceback=False,
    version=str(repo / "tools/gui-version.txt"),
)
if onedir:
    coll = COLLECT(exe, a.binaries, a.datas, strip=False, upx=False, name="STM32FC-GUI")
