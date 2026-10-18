import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parent
SOURCE = ROOT / "launcher"
OUTPUT = ROOT / "Cubli.exe"

result = subprocess.run(
    ["go", "build", "-trimpath", "-ldflags", "-H=windowsgui -s -w", "-o", str(OUTPUT), "."],
    cwd=SOURCE,
)
if result.returncode != 0:
    sys.exit("Failed")

print(f"已生成 {OUTPUT}（{OUTPUT.stat().st_size} 字节）")
