"""Compile and exercise the actual portable firmware modules without hardware."""

from pathlib import Path
import os
import shutil
import subprocess
import tempfile


def main() -> None:
    tests = Path(__file__).resolve().parent
    firmware = tests.parent
    compiler = shutil.which(os.environ.get("CXX", "g++"))
    if compiler is None:
        raise SystemExit("Install g++ or set CXX to a compatible C++ compiler.")
    with tempfile.TemporaryDirectory(prefix="omni-host-tests-") as build:
        binary = Path(build) / ("host_tests.exe" if os.name == "nt" else "host_tests")
        command = [
            compiler, "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
            "-I", str(tests / "stubs"), "-I", str(firmware / "include"),
            str(tests / "host_tests.cpp"),
            *(str(firmware / "src" / name) for name in
              ("kinematics.cpp", "tracking.cpp", "pid_speed.cpp")),
            "-o", str(binary),
        ]
        subprocess.run(command, check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
