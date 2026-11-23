#!/usr/bin/env bash
# pode-drive main을 roscamp-nav2fleet main으로 옮긴다. learning/(RL 학습)만 뺀다
#   tools/sync_nav2fleet.sh NAV2FLEET_CLONE          # 합친 결과를 그 클론의 sync/pode-drive 브랜치에 둔다(덮어씀)
#   tools/sync_nav2fleet.sh NAV2FLEET_CLONE --push   # 관제 시험이 통과하면 nav2fleet main에 올린다(fast-forward만)
# 사용자 결정 2026-11-23: nav2fleet 얼림을 풀고 pode-drive와 맞춘다. 원본은 pode-drive다
# 클론의 작업 트리는 건드리지 않는다(임시 worktree에서 합친다)
set -euo pipefail

usage() { echo "usage: sync_nav2fleet.sh NAV2FLEET_CLONE [--push]"; exit 2; }
[ $# -ge 1 ] && [ $# -le 2 ] || usage
push=${2:-}
[ -z "$push" ] || [ "$push" = --push ] || usage
dst=$(cd "$1" 2> /dev/null && pwd -P) || { echo "$1: no such directory"; exit 1; }
SRC_URL=${SRC_URL:-https://github.com/PJW-1/roscamp-pode-drive}
DROP=(learning)

# 가져오는 곳과 올리는 곳 모두 nav2fleet이어야 한다
for url in "$(git -C "$dst" remote get-url origin 2> /dev/null)" \
           "$(git -C "$dst" remote get-url --push origin 2> /dev/null)"; do
  [[ $url =~ roscamp-nav2fleet(\.git)?/?$ ]] || { echo "$dst: origin is '$url', not roscamp-nav2fleet"; exit 1; }
done

git -C "$dst" fetch -q origin main
git -C "$dst" fetch -q "$SRC_URL" main
src=$(git -C "$dst" rev-parse FETCH_HEAD)
has_drop() { [ -n "$(git -C "$dst" ls-tree --name-only "$1" -- "${DROP[@]}")" ]; }
synced=0
git -C "$dst" merge-base --is-ancestor "$src" origin/main && synced=1
if [ $synced = 1 ] && ! has_drop origin/main; then
  echo "nav2fleet main already has pode-drive ${src:0:7}"; exit 0
fi

tmp=$(mktemp -d)
cleanup() { git -C "$dst" worktree remove --force "$tmp" 2> /dev/null || true; rm -rf "$tmp"
            git -C "$dst" worktree prune; }
trap cleanup EXIT
git -C "$dst" worktree add -q --detach "$tmp" origin/main
cd "$tmp"

if [ $synced = 0 ]; then
  # 합치고 learning/을 지운다. learning/의 수정-삭제 충돌은 지우는 쪽으로 풀린다
  git merge --no-ff --no-commit -q "$src" > /dev/null 2>&1 || true
  git rev-parse -q --verify MERGE_HEAD > /dev/null || { echo "merge did not start"; exit 1; }
  message="Sync from roscamp-pode-drive main ${src:0:7} (without ${DROP[*]}/)"
else
  message="Drop ${DROP[*]}/ (RL training lives only in roscamp-pode-drive)"
fi
git rm -r -q -f --ignore-unmatch -- "${DROP[@]}"  # 합치며 새로 들어온 파일도 지운다
left=$(git diff --name-only --diff-filter=U)
if [ -n "$left" ]; then
  echo "conflicts outside ${DROP[*]}; fix by hand:"; echo "$left"
  git merge --abort; exit 1
fi
git commit -q -m "$message"
result=$(git rev-parse HEAD)
echo "nav2fleet $(git rev-parse --short origin/main) + pode-drive ${src:0:7} -> ${result:0:7}"

if [ -z "$push" ]; then
  git branch -f sync/pode-drive "$result"
  echo "result on branch sync/pode-drive in $dst (not pushed)"; exit 0
fi
# 올리기 전에 관제 시험을 돌린다
(cd pc/fleet_core && python3 -m pytest -q -p no:cacheprovider test > "$tmp.pytest.log" 2>&1) || {
  echo "fleet_core tests failed; not pushed (log $tmp.pytest.log)"; git branch -f sync/pode-drive "$result"; exit 1; }
rm -f "$tmp.pytest.log"
# 시험한 그 커밋만 올린다. learning/이 없는지 한 번 더 본다
! has_drop "$result" || { echo "$result still has ${DROP[*]}/; not pushed"; exit 1; }
git push -q origin "$result:refs/heads/main"
echo "pushed nav2fleet main ${result:0:7}"
