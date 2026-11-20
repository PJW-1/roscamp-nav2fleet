#!/usr/bin/env bash
# make_pinky_urdf.sh를 임시 폴더에 돌려 결과를 확인한다. WSL에서 실행.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
T="$(mktemp -d)"
trap 'rm -rf "$T"' EXIT
fail() { echo "FAIL: $*" >&2; exit 1; }
set +u; source /opt/ros/jazzy/setup.bash; set -u
MK="$here/make_pinky_urdf.sh"
SRC0="${PINKY_DESCRIPTION_DIR:-$HOME/pinky_deps_ws/src/pinky_pro/pinky_description}"

# 모든 <mesh filename>을 확인한다: 인자 = urdf, 접두사(비면 상대 경로 모드)
check_meshes() {
python3 - "$1" "${2:-}" <<'PY' || fail "mesh check ($1 ${2:-})"
import sys, xml.etree.ElementTree as ET
from pathlib import Path
u, prefix = Path(sys.argv[1]), sys.argv[2]
r = ET.parse(u).getroot()
m = [e.get("filename") for e in r.iter("mesh")]
assert m, "no meshes"
for f in m:
    if prefix:
        assert f.startswith(prefix + "/meshes/"), f"not prefixed: {f}"
        rel = f[len(prefix) + 1:]
    else:
        assert not f.startswith("/") and ":" not in f, f"not relative: {f}"
        rel = f
    assert (u.parent / rel).is_file(), f"missing mesh {f}"
j = {e.get("name") for e in r.iter("joint")}
for n in ("l_wheel_joint", "r_wheel_joint"):
    assert n in j, f"missing {n}"
PY
}

bash "$MK" "$T/pinky" >/dev/null
U="$T/pinky/pinky.urdf"
check_urdf "$U" >/dev/null || fail "check_urdf"
! grep -q 'package://' "$U" || fail "package:// left"
check_meshes "$U"

# 절대 경로 옵션
bash "$MK" "$T/abs" --abs-prefix C:/isaac_assets/pinky >/dev/null
check_meshes "$T/abs/pinky.urdf" C:/isaac_assets/pinky

# 실패 시험은 소스 패키지의 임시 복사본을 쓴다 (원본은 건드리지 않는다)
cp -rL "$SRC0" "$T/src"

# 메시가 없으면 실패하고 이전 출력은 그대로
bash "$MK" "$T/keep" >/dev/null
echo sentinel > "$T/keep/meshes/sentinel.txt"
before="$(cd "$T/keep" && find . -type f | sort | xargs md5sum)"
rm "$T/src/meshes/visual/$(ls "$T/src/meshes/visual" | head -n 1)"
if PINKY_DESCRIPTION_DIR="$T/src" bash "$MK" "$T/keep" >/dev/null 2>&1; then fail "missing mesh did not fail"; fi
after="$(cd "$T/keep" && find . -type f | sort | xargs md5sum)"
[ "$before" = "$after" ] || fail "previous output changed after failed run"

# OUT_DIR가 소스 패키지 안이면 거부하고 소스는 그대로
cp -rL "$SRC0" "$T/src2"
sbefore="$(cd "$T/src2" && find . -type f | sort | xargs md5sum)"
for o in "$T/src2" "$T/src2/out" "$T/src2/meshes"; do
  if PINKY_DESCRIPTION_DIR="$T/src2" bash "$MK" "$o" >/dev/null 2>&1; then fail "OUT_DIR $o not refused"; fi
done
safter="$(cd "$T/src2" && find . -type f | sort | xargs md5sum)"
[ "$sbefore" = "$safter" ] || fail "source package changed"
echo "PASS"
