#!/usr/bin/env bash
# make_pinky_urdf.sh를 임시 폴더에 돌려 결과를 확인한다. WSL에서 실행.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
T="$(mktemp -d)"
trap 'rm -rf "$T"' EXIT
fail() { echo "FAIL: $*" >&2; exit 1; }
set +u; source /opt/ros/jazzy/setup.bash; set -u

bash "$here/make_pinky_urdf.sh" "$T/pinky" >/dev/null
U="$T/pinky/pinky.urdf"
check_urdf "$U" >/dev/null || fail "check_urdf"
! grep -q 'package://' "$U" || fail "package:// left"
python3 - "$U" <<'PY' || exit 1
import sys, xml.etree.ElementTree as ET
from pathlib import Path
u = Path(sys.argv[1]); r = ET.parse(u).getroot()
m = [e.get("filename") for e in r.iter("mesh")]
assert m, "no meshes"
for f in m:
    assert (u.parent / f).is_file(), f"missing mesh {f}"
j = {e.get("name") for e in r.iter("joint")}
for n in ("l_wheel_joint", "r_wheel_joint"):
    assert n in j, f"missing {n}"
PY

# 절대 경로 옵션
bash "$here/make_pinky_urdf.sh" "$T/abs" --abs-prefix C:/isaac_assets/pinky >/dev/null
grep -q '"C:/isaac_assets/pinky/meshes/' "$T/abs/pinky.urdf" || fail "abs prefix"
echo "PASS"
