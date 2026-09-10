#!/usr/bin/env python3
"""Run the actual firmware state machine on a host with clock/GPIO/NVS stubs."""
from pathlib import Path
import subprocess
import tempfile

firmware = Path(__file__).resolve().parents[1]
source = (firmware / "src/main.cpp").read_text()
start = source.index("enum class SysState")
end = source.index("// -------------------------------------------------------------- scan loop")
with tempfile.TemporaryDirectory(prefix="bluedoor-presence-") as tmp:
    generated = Path(tmp) / "machine.inc"
    generated.write_text(source[start:end])
    for pulse_enabled in (0, 1):
        for uconnect_enabled in (0, 1):
            binary = Path(tmp) / f"presence-{pulse_enabled}-{uconnect_enabled}"
            subprocess.run([
                "g++", "-std=c++17", "-Wall", "-Wextra", "-Wno-unused-function",
                "-DDETECT_BLE=1", f"-DPULSE_ENABLED={pulse_enabled}",
                f"-DUCONNECT_HINT_ENABLED={uconnect_enabled}",
                f"-I{tmp}", f"-I{firmware / 'include'}",
                str(firmware / "tests/presence_test.cpp"), "-o", str(binary),
            ], check=True)
            subprocess.run([str(binary)], check=True)
