#!/usr/bin/env python3
"""Run the portable firmware suite with GCC/G++ on Linux (no hardware access)."""
import argparse
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[1]
BUILD=ROOT/"build/host-linux"
SHARED=[str(p.relative_to(ROOT)) for part in ("control","modes","estimation") for p in sorted((ROOT/"src"/part).glob("*.cpp"))]
CORE=[f"src/core/{name}.cpp" for name in ("arming","failsafe","scheduler","log_ring","log_frame")]
REG=SHARED+CORE+["tests/regression.cpp","src/drivers/crsf.cpp","src/drivers/ublox.cpp"]
TARGETS={
    "native":SHARED+CORE+["src/main_native.cpp","src/hal/native/hal_native.cpp"],
    "sitl":SHARED+["src/main_sitl.cpp","src/sitl/aircraft.cpp","src/sitl/sensors.cpp"],
    "regression":REG,"bench":REG,"sd_timeout":["tests/sd_timeout.cpp"],
    "sd_logging":["tests/async_storage.cpp","src/core/sd_bin_log.cpp","src/core/log_ring.cpp","src/core/log_frame.cpp"],
    "bmi_driver":["tests/bmi270_driver.cpp","src/drivers/bmi270.cpp","src/drivers/imu_v2.cpp","src/estimation/imu_prep.cpp","src/estimation/ahrs.cpp"],
    "bmp_driver":["tests/bmp581_driver.cpp","src/drivers/bmp581.cpp"],
    "bmm_driver":["tests/bmm350_driver.cpp","src/drivers/bmm350.cpp"],
    "heading":SHARED+["tests/magnetic_heading.cpp"],
    "improvements":SHARED+["tests/control_improvements.cpp"],
    "scenarios":SHARED+["tests/flight_scenarios.cpp","src/sitl/aircraft.cpp","src/sitl/sensors.cpp"],
}


def run(command, log):
    with log.open("w") as stream:
        result=subprocess.run(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
    if result.returncode:
        print(log.read_text()); raise SystemExit(result.returncode)
    return log.read_text()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target",action="append",choices=TARGETS)
    args=parser.parse_args(); BUILD.mkdir(parents=True,exist_ok=True); count=0
    for name in args.target or TARGETS:
        flags=[]
        if name=="regression": flags += ["-DFC_FLIGHT_ENABLED=1","-DFC_SD_LOGGING=1"]
        vendor={"bmi_driver":("bmi270",("bmi2","bmi270")),"bmp_driver":("bmp5",("bmp5",)),"bmm_driver":("bmm350",("bmm350",))}.get(name)
        if vendor:
            lib,files=vendor; flags += [f"-Ilib/{lib}/src"]
            for file in files:
                obj=BUILD/f"{file}.o"
                run(["gcc","-O2","-c",f"lib/{lib}/src/{file}.c",f"-Ilib/{lib}/src","-o",str(obj)],BUILD/f"{file}-build.txt")
                flags.append(str(obj))
        run(["g++","-std=c++17","-O2","-Wall","-Wextra","-Itests/stubs",*TARGETS[name],*flags,"-o",str(BUILD/name)],BUILD/f"{name}-build.txt")
        for test_args in (("--check",),("--assist-check",)) if name=="sitl" else ((),):
            suffix="-assist" if test_args==("--assist-check",) else ""
            output=run([str(BUILD/name),*test_args],BUILD/f"{name}{suffix}-results.txt")
            count+=output.count("[PASS]")
            print(f"{name} {' '.join(test_args)}: PASS",flush=True)
    print(f"{count} C/C++ checks passed",flush=True)


if __name__=="__main__": main()
