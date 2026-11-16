#!/usr/bin/env bash
# PC에서 저장소의 커밋된 HEAD를 로봇에 복사한다:  tools/real_pode/deploy.sh pinky@192.168.0.201
# 패키지 -> ROBOT:~/nav2fleet_ws/src/<저장소 경로>/, 실행 묶음(지도 포함) -> ROBOT:~/nav2fleet_real/
# 빌드와 실행은 하지 않는다. 지우기는 각 패키지 폴더 안에서만 한다(logs, params는 남는다).
set -euo pipefail
[ $# -eq 1 ] || { echo "usage: deploy.sh ROBOT_HOST"; exit 2; }
host=$1
repo=$(git -C "$(dirname "$(readlink -f "$0")")" rev-parse --show-toplevel)
packages=(interfaces/fleet_interfaces pode/pode_fsm pode/pode_nav_config
  pode/pode_avoidance pode/pode_peer_obstacles pode/pode_scan_filter pode/pode_dock)
kit=tools/real_pode

# ---- 커밋된 내용만 꺼낸다 ----
tmp=$(mktemp -d)
trap 'rm -rf -- "$tmp"' EXIT
git -C "$repo" archive HEAD "${packages[@]}" "$kit" | tar -x -C "$tmp"
if [ -n "$(git -C "$repo" status --porcelain -- "${packages[@]}" "$kit")" ]; then
  echo "주의: 커밋하지 않은 변경은 복사하지 않는다(HEAD만 간다)"
fi
echo "HEAD $(git -C "$repo" rev-parse --short HEAD) -> $host"

# ---- 복사 ----
ssh "$host" 'mkdir -p ~/nav2fleet_ws/src/interfaces ~/nav2fleet_ws/src/pode ~/nav2fleet_real'
for pkg in "${packages[@]}"; do
  rsync -a --delete "$tmp/$pkg/" "$host:~/nav2fleet_ws/src/$pkg/"
done
rsync -a "$tmp/$kit/" "$host:~/nav2fleet_real/"

echo "복사 끝. 로봇에서 빌드한다:"
echo "  cd ~/nav2fleet_ws && colcon build"
