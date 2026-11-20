#!/usr/bin/env bash
# Pode (Pinky Pro) 모델을 Isaac Lab(Windows)에서 읽을 수 있는 URDF + 메시 묶음으로 만든다.
# WSL에서 실행한다. 사용법은 --help.
set -euo pipefail

usage() {
  cat <<'USAGE'
usage: make_pinky_urdf.sh [OUT_DIR] [--abs-prefix PREFIX]

  OUT_DIR       출력 폴더 (기본 /mnt/c/isaac_assets/pinky = Windows C:\isaac_assets\pinky)
  --abs-prefix  메시 경로를 PREFIX/meshes/... 절대 경로로 쓴다 (예: C:/isaac_assets/pinky).
                생략하면 URDF 파일 기준 상대 경로(meshes/...)다.

환경 변수
  PINKY_DESCRIPTION_DIR  pinky_description 패키지 폴더
                         (기본 ~/pinky_deps_ws/src/pinky_pro/pinky_description)

출력: OUT_DIR/pinky.urdf, OUT_DIR/meshes/{visual,collision}/*
xacro 입력은 urdf/robot.urdf.xacro (is_sim:=false). Gazebo 시뮬(pode_sim.urdf.xacro)과 같은 로봇이다.
package:// 가 남거나 메시가 없거나 check_urdf가 실패하면 0이 아닌 값으로 끝난다.
USAGE
}

OUT=""
ABS=""
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --abs-prefix) [ $# -ge 2 ] || { echo "error: --abs-prefix needs a value" >&2; exit 2; }; ABS="${2%/}"; shift 2 ;;
    -*) echo "error: unknown option $1" >&2; usage >&2; exit 2 ;;
    *) [ -z "$OUT" ] || { echo "error: more than one OUT_DIR" >&2; exit 2; }; OUT="$1"; shift ;;
  esac
done
OUT="${OUT:-/mnt/c/isaac_assets/pinky}"
SRC="${PINKY_DESCRIPTION_DIR:-$HOME/pinky_deps_ws/src/pinky_pro/pinky_description}"

[ -f "$SRC/urdf/robot.urdf.xacro" ] || { echo "error: $SRC/urdf/robot.urdf.xacro not found" >&2; exit 1; }
if ! command -v xacro >/dev/null || ! command -v check_urdf >/dev/null; then
  set +u; source /opt/ros/jazzy/setup.bash; set -u
fi
command -v xacro >/dev/null || { echo "error: xacro not found" >&2; exit 1; }
command -v check_urdf >/dev/null || { echo "error: check_urdf not found (liburdfdom-tools)" >&2; exit 1; }

# $(find pinky_description) 가 소스 폴더를 가리키도록 임시 ament 접두사를 만든다
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
mkdir -p "$TMP/share/ament_index/resource_index/packages"
touch "$TMP/share/ament_index/resource_index/packages/pinky_description"
ln -s "$(readlink -f "$SRC")" "$TMP/share/pinky_description"

mkdir -p "$OUT"
rm -rf "$OUT/meshes"
AMENT_PREFIX_PATH="$TMP:${AMENT_PREFIX_PATH:-}" xacro "$SRC/urdf/robot.urdf.xacro" is_sim:=false > "$OUT/pinky.urdf"

python3 - "$OUT" "$SRC" "$ABS" <<'PY'
import re, shutil, sys
from pathlib import Path
out, src, abs_prefix = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
urdf = out / "pinky.urdf"
text = urdf.read_text(encoding="utf-8")
n = 0
def repl(m):
    global n
    rel = m.group(1)  # meshes/visual/x.dae
    s = src / rel
    if not s.is_file():
        sys.exit(f"error: missing mesh {s}")
    d = out / rel
    d.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(s, d)
    n += 1
    return f'"{abs_prefix}/{rel}"' if abs_prefix else f'"{rel}"'
text = re.sub(r'"package://pinky_description/([^"]+)"', repl, text)
if "package://" in text:
    sys.exit("error: package:// URI left in " + str(urdf))
urdf.write_text(text, encoding="utf-8")
if not abs_prefix:
    bad = [f for f in re.findall(r'<mesh filename="([^"]+)"', text) if not (out / f).is_file()]
    if bad:
        sys.exit("error: unresolved meshes: " + ", ".join(bad))
print(f"meshes: {n} references, {sum(1 for _ in (out / 'meshes').rglob('*') if _.is_file())} files")
PY

check_urdf "$OUT/pinky.urdf" >"$TMP/check.txt" || { cat "$TMP/check.txt" >&2; echo "error: check_urdf failed" >&2; exit 1; }
tail -n 5 "$TMP/check.txt"
echo "OK: $OUT/pinky.urdf"
