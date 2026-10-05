#!/usr/bin/env python3
"""Build the standalone Windows x64 dashboard and a shareable ZIP.

Run in a Python 3.12 x64 venv with gui-requirements.txt installed.
Only the build needs internet, Python and PyInstaller. No firmware is bundled.
"""
import argparse
import hashlib
from importlib import metadata
import json
import os
from pathlib import Path
import platform
import shutil
import struct
import subprocess
import sys
import tarfile
import tkinter
import urllib.request
import zipfile

REPO = Path(__file__).resolve().parents[1]
ASSETS = REPO / "build/gui-assets"
DOWNLOADS = REPO / "build/gui-downloads"
RELEASE = "2026.10.05"
# Original publisher archives, including the exact libusb revision documented
# in dfu-util's README-bin.txt (not the older libusb 1.0.24 release tarball).
ARCHIVES = {
    "dfu-util-0.11-binaries.tar.xz": (
        "https://dfu-util.sourceforge.net/releases/dfu-util-0.11-binaries.tar.xz",
        "6450de30a7dcd8d8c1273f43f0b153f054fd24d85f7f38296b1ad8edbd2ddb25"),
    "dfu-util-0.11.tar.gz": (
        "https://dfu-util.sourceforge.net/releases/dfu-util-0.11.tar.gz",
        "b4b53ba21a82ef7e3d4c47df2952adf5fa494f499b6b0b57c58c5d04ae8ff19e"),
    "libusb-1a90627.tar.gz": (
        "https://codeload.github.com/libusb/libusb/tar.gz/1a90627",
        "5b8158fa497e1ae8fcb03592bb4b27fa9b04f4acbd3fabd6bf34c05143b28f3e"),
    "pyserial-3.5-LICENSE.txt": (
        "https://raw.githubusercontent.com/pyserial/pyserial/v3.5/LICENSE.txt",
        "f91cb9813de6a5b142b8f7f2dede630b5134160aedaeaf55f4d6a7e2593ca3f3"),
}


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def archive(name):
    url, expected = ARCHIVES[name]
    path = DOWNLOADS / name
    if not path.is_file():
        print(f"Downloading {name}", flush=True)
        with urllib.request.urlopen(url, timeout=60) as response:
            data = response.read()
        if hashlib.sha256(data).hexdigest() != expected:
            raise RuntimeError(f"Checksum mismatch: {name}")
        path.write_bytes(data)
    if sha256(path) != expected:
        raise RuntimeError(f"Cached archive checksum mismatch: {path}")
    return path


def member(archive_path, name, destination):
    # Copy only explicitly named regular files; never extract arbitrary paths.
    with tarfile.open(archive_path) as tar:
        entry = tar.getmember(name)
        if not entry.isfile():
            raise RuntimeError(f"Expected regular archive member: {name}")
        destination.write_bytes(tar.extractfile(entry).read())


def prepare():
    vendor = ASSETS / "vendor/dfu-util"
    licenses = ASSETS / "licenses"
    sources = ASSETS / "sources"
    for path in (DOWNLOADS, vendor, licenses, sources):
        path.mkdir(parents=True, exist_ok=True)
    binary_archive = archive("dfu-util-0.11-binaries.tar.xz")
    for name in ("dfu-util.exe", "libusb-1.0.dll"):
        member(binary_archive, "dfu-util-0.11-binaries/win64/" + name, vendor / name)
    member(binary_archive, "dfu-util-0.11-binaries/COPYING", licenses / "dfu-util-GPL-2.txt")
    member(binary_archive, "dfu-util-0.11-binaries/README-bin.txt", sources / "dfu-util-binary-build.txt")
    for name in ("dfu-util-0.11.tar.gz", "libusb-1a90627.tar.gz"):
        shutil.copy2(archive(name), sources / name)
    member(archive("libusb-1a90627.tar.gz"), "libusb-1a90627/COPYING", licenses / "libusb-LGPL-2.1.txt")
    shutil.copy2(Path(sys.base_prefix) / "LICENSE.txt", licenses / "Python-LICENSE.txt")
    tcl_path = Path(tkinter.Tcl().eval("info library"))
    shutil.copy2(tcl_path.parent / "tk8.6/license.terms", licenses / "Tcl-Tk-license.terms")
    # pySerial's 3.5 wheel omits its license; fetch the exact tagged copy.
    shutil.copy2(archive("pyserial-3.5-LICENSE.txt"), licenses / "pyserial-LICENSE.txt")
    for package in ("pyinstaller",):
        dist = metadata.distribution(package)
        found = [f for f in dist.files if f.name.lower().startswith(("license", "copying"))]
        if not found:
            raise RuntimeError(f"Missing license for {package}")
        for n, path in enumerate(found):
            shutil.copy2(dist.locate_file(path), licenses / f"{package}-{n}-{path.name}")
    shutil.copy2(REPO / "docs/GUI_THIRD_PARTY.txt", licenses / "THIRD-PARTY-NOTICES.txt")
    # Include editable GUI source and packaging recipe alongside DFU sources.
    with zipfile.ZipFile(sources / "STM32FC-GUI-source.zip", "w", zipfile.ZIP_DEFLATED) as z:
        for path in [*(REPO / "tools").glob("gui*"), REPO / "tools/enter_dfu.py",
                     REPO / "tools/build_gui.py", REPO / "tools/STM32FC-GUI.spec",
                     REPO / "docs/GUI_WINDOWS.md", REPO / "docs/GUI_THIRD_PARTY.txt",
                     REPO / "tests/gui_parser.py", REPO / "tests/gui_connection.py",
                     REPO / "tests/dfu_helper.py"]:
            if path.is_file():
                z.write(path, path.relative_to(REPO))
    info = {"release": RELEASE, "platform": "Windows x64", "python": platform.python_version(),
            "packages": {p: metadata.version(p) for p in ("pyinstaller", "pyinstaller-hooks-contrib", "pyserial")},
            "vendor_archives": {n: {"url": u, "sha256": h} for n, (u, h) in ARCHIVES.items()},
            "vendor_files": {p.name: sha256(p) for p in vendor.iterdir()},
            "gui_sha256": sha256(REPO / "tools/gui.py")}
    (ASSETS / "build-info.json").write_text(json.dumps(info, indent=2) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--onedir", action="store_true", help="diagnostic folder build before the one-file release")
    args = parser.parse_args()
    if sys.platform != "win32" or struct.calcsize("P") != 8 or platform.machine().upper() not in ("AMD64", "X86_64"):
        parser.error("Build using 64-bit x86 Python on Windows.")
    prepare()
    env = os.environ.copy()
    env["STM32FC_GUI_ONEDIR"] = "1" if args.onedir else "0"
    dist = REPO / ("build/gui-onedir" if args.onedir else "dist")
    subprocess.run([sys.executable, "-m", "PyInstaller", "--noconfirm", "--clean",
                    "--distpath", str(dist), "--workpath", str(REPO / "build/gui-pyinstaller"),
                    str(REPO / "tools/STM32FC-GUI.spec")], cwd=REPO, env=env, check=True)
    if args.onedir:
        print(dist / "STM32FC-GUI/STM32FC-GUI.exe")
        return
    exe = dist / "STM32FC-GUI.exe"
    digest = sha256(exe)
    (dist / "STM32FC-GUI.exe.sha256").write_text(f"{digest}  {exe.name}\n", encoding="ascii")
    release_zip = dist / f"STM32FC-GUI-{RELEASE}-Windows-x64.zip"
    with zipfile.ZipFile(release_zip, "w", zipfile.ZIP_DEFLATED) as z:
        z.write(exe, exe.name)
        z.write(dist / "STM32FC-GUI.exe.sha256", "STM32FC-GUI.exe.sha256")
        z.write(REPO / "docs/GUI_WINDOWS.md", "README.md")
        z.write(ASSETS / "build-info.json", "build-info.json")
        for folder in ("licenses", "sources"):
            for path in sorted((ASSETS / folder).rglob("*")):
                if path.is_file():
                    z.write(path, path.relative_to(ASSETS))
    print(f"\nEXE: {exe}\nSHA256: {digest}\nPortable distribution: {release_zip}")


if __name__ == "__main__":
    main()
