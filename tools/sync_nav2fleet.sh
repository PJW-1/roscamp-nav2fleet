#!/usr/bin/env bash
# pode-drive main을 roscamp-nav2fleet main으로 옮긴다. learning/(RL 학습)만 뺀다
#   tools/sync_nav2fleet.sh NAV2FLEET_CLONE          # 합친 결과를 그 클론의 sync/pode-drive 브랜치에 둔다
#   tools/sync_nav2fleet.sh NAV2FLEET_CLONE --push   # 관제 시험이 통과하면 nav2fleet main에 올린다(fast-forward만)
# 사용자 결정 2026-11-23: nav2fleet 얼림을 풀고 pode-drive와 맞춘다. 원본은 pode-drive다
# 클론의 작업 트리는 건드리지 않는다(임시 worktree에서 합친다)
set -euo pipefail

usage() { echo "usage: sync_nav2fleet.sh NAV2FLEET_CLONE [--push]"; exit 2; }
[ $# -ge 1 ] && [ $# -le 2 ] || usage
dst=$1 push=${2:-}
[ -z "$push" ] || [ "$push" = --push ] || usage
SRC_URL=${SRC_URL:-https://github.com/PJW-1/roscamp-pode-drive}
DROP=(learning)

url=$(git -C "$dst" remote get-url origin 2> /dev/null) || { echo "$dst: not a git clone with origin"; exit 1; }
[[ $url =~ roscamp-nav2fleet(\.git)?/?$ ]] || { echo "$dst: origin is $url, not roscamp-nav2fleet"; exit 1; }

git -C "$dst" fetch -q origin main
git -C "$dst" fetch -q "$SRC_URL" main
src=$(git -C "$dst" rev-parse FETCH_HEAD)
if git -C "$dst" merge-base --is-ancestor "$src" origin/main; then
  echo "nav2fleet main already has pode-drive ${src:0:7}"; exit 0
fi

tmp=$(mktemp -d)
cleanup() { git -C "$dst" worktree remove --force "$tmp" 2> /dev/null || true; rm -rf "$tmp"; }
trap cleanup EXIT
git -C "$dst" worktree add -q --detach "$tmp" origin/main
cd "$tmp"

# 합치고 learning/을 지운다. learning/의 수정-삭제 충돌은 지우는 쪽으로 풀린다
git merge --no-ff --no-commit -q "$src" > /dev/null 2>&1 || true
git rev-parse -q --verify MERGE_HEAD > /dev/null || { echo "merge did not start"; git merge --abort 2> /dev/null; exit 1; }
git rm -r -q -f --ignore-unmatch -- "${DROP[@]}"  # 합치며 새로 들어온 파일도 지운다
left=$(git diff --name-only --diff-filter=U)
if [ -n "$left" ]; then
  echo "conflicts outside ${DROP[*]}; fix by hand:"; echo "$left"
  git merge --abort; exit 1
fi
git commit -q -m "Sync from roscamp-pode-drive main ${src:0:7} (without ${DROP[*]}/)"
echo "merged pode-drive ${src:0:7} into nav2fleet $(git rev-parse --short HEAD^1) -> $(git rev-parse --short HEAD)"

if [ -z "$push" ]; then
  git branch -f sync/pode-drive HEAD
  echo "result on branch sync/pode-drive in $dst (not pushed)"; exit 0
fi
# 올리기 전에 관제 시험을 돌린다
(cd pc/fleet_core && python3 -m pytest -q -p no:cacheprovider test > "$tmp.pytest.log" 2>&1) || {
  echo "fleet_core tests failed; not pushed (log $tmp.pytest.log)"; git branch -f sync/pode-drive HEAD; exit 1; }
rm -f "$tmp.pytest.log"
git push -q origin HEAD:main
echo "pushed nav2fleet main $(git rev-parse --short HEAD)"
