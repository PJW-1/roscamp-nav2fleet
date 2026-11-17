#!/usr/bin/env bash
# ZoneTraffic 정리 비교: 기준(~/zone_baseline/acfa45b)과 지금 작업 폴더의 출력을 비교한다.
#   [OUT=<결과 폴더>] bash tools/zone_compare.sh [작업 폴더] [기준 폴더]
# 출력: 표마다 diff 줄 수, fleet_core / fake_robots / tools 시험 요약. 결과 파일은 $OUT에 남는다.
set -u
WT=${1:-$(cd "$(dirname "$0")/.." && pwd)}
B=${2:-$HOME/zone_baseline/acfa45b}
OUT=${OUT:-$HOME/zone_baseline/last_run}
rm -rf "$OUT" && mkdir -p "$OUT"
cd "$WT" || exit 2
export PYTHONUTF8=1
R=$(cygpath -m "$WT")
PP="$R/pc/fleet_core;$R/sim/fake_robots;$R/tools"
start=$(date +%s)

python tools/zone_trace.py > "$OUT/trace.txt" 2>/dev/null &
python tools/situations.py -l v7 d2 d c --avoid all --seeds 1 2 3 > "$OUT/situations.md" 2>/dev/null &
python tools/situations.py --store -l d2 --seeds 1 2 3 > "$OUT/store.md" 2>/dev/null &
python tools/situations.py --store -l v7 d2 --seeds 1 2 3 > "$OUT/store_v7_d2.md" 2> "$OUT/store_v7_d2.err" &
python tools/order_situations.py --seeds 1 2 3 -l v7 > "$OUT/order_v7.md" 2>/dev/null &
python tools/order_situations.py --seeds 1 2 3 -l d2 > "$OUT/order_d2.md" 2>/dev/null &
(cd pc/fleet_core && ZONE_TICK_TRACE="$(cygpath -m "$OUT")/suite_trace.json" \
  python -m pytest -q test -p no:cacheprovider > "$OUT/fleet_core.txt" 2>&1) &
(PYTHONPATH="$PP" python -m pytest -q sim/fake_robots/test -p no:cacheprovider \
  > "$OUT/fake_robots.txt" 2>&1) &
(PYTHONPATH="$PP" python -m pytest -q tools/test -p no:cacheprovider > "$OUT/tools.txt" 2>&1) &
wait

for name in trace.txt situations.md store.md order_v7.md order_d2.md; do
  echo "$name diff lines: $(diff "$B/$name" "$OUT/$name" | wc -l)"
done
echo "store -l v7 d2 crash (last stderr line): $(tail -1 "$OUT/store_v7_d2.err")"
echo "  baseline: $(tail -1 "$B/store_v7_d2_crash.err")"
python - "$B/suite_trace.json" "$OUT/suite_trace.json" <<'EOF'
import json, sys
old, new = (json.load(open(p)) for p in sys.argv[1:3])
changed = [k for k in old if old[k] != new.get(k)]
extra = [k for k in new if k not in old and new[k]]
print(f"suite_trace: {len(old)} tests in baseline, changed {len(changed)}, "
      f"new tests with worlds {len(extra)}")
for k in changed[:10] + extra[:10]:
    print("   ", k)
EOF
echo "fleet_core: $(tail -1 "$OUT/fleet_core.txt")"
echo "fake_robots: $(tail -1 "$OUT/fake_robots.txt")"
echo "tools: $(tail -1 "$OUT/tools.txt")"
echo "elapsed $(( $(date +%s) - start ))s"
