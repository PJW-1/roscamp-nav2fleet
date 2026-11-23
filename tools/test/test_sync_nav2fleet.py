"""tools/sync_nav2fleet.sh 시험. 로컬 bare 저장소 둘로 pode-drive -> nav2fleet 동기화를 흉내 낸다."""
import os
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "tools" / "sync_nav2fleet.sh"
ENV = dict(os.environ, GIT_AUTHOR_NAME="t", GIT_AUTHOR_EMAIL="t@t", GIT_COMMITTER_NAME="t",
           GIT_COMMITTER_EMAIL="t@t", GIT_CONFIG_GLOBAL="/dev/null", GIT_CONFIG_NOSYSTEM="1")


def _git(cwd, *args):
    return subprocess.run(["git", *args], cwd=cwd, env=ENV, check=True, capture_output=True,
                          text=True).stdout.strip()


def _write(repo, files):
    for name, text in files.items():
        path = repo / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)


def _commit(repo, files, message, remove=()):
    _write(repo, files)
    for name in remove:
        _git(repo, "rm", "-r", "-q", name)
    _git(repo, "add", "-A")
    _git(repo, "commit", "-q", "-m", message)


def _setup(tmp_path):
    # pode-drive 작업 사본(src), 그 bare(src.git), nav2fleet bare와 클론(dst)
    src = tmp_path / "src"
    src.mkdir()
    _git(src, "init", "-q", "-b", "main")
    _commit(src, {"pc/fleet_core/test/test_ok.py": "def test_ok():\n    assert True\n",
                  "README.md": "a\n"}, "base")
    src_bare = tmp_path / "roscamp-pode-drive.git"
    _git(tmp_path, "clone", "-q", "--bare", str(src), str(src_bare))
    dst_bare = tmp_path / "roscamp-nav2fleet.git"
    _git(tmp_path, "clone", "-q", "--bare", str(src), str(dst_bare))
    _git(src, "remote", "add", "origin", str(src_bare))
    dst = tmp_path / "dst"
    _git(tmp_path, "clone", "-q", str(dst_bare), str(dst))
    return src, dst, dst_bare


def _push_src(src, files, message, remove=()):
    _commit(src, files, message, remove)
    _git(src, "push", "-q", "origin", "main")


def _run(dst, tmp_path, *args):
    env = dict(ENV, SRC_URL=str(tmp_path / "roscamp-pode-drive.git"))
    return subprocess.run(["bash", str(SCRIPT), str(dst), *args], env=env, capture_output=True,
                          text=True, timeout=120)


def _tree(bare, ref="main"):
    return set(_git(bare, "ls-tree", "-r", "--name-only", ref).splitlines())


def test_push_drops_learning_and_keeps_the_rest(tmp_path):
    src, dst, dst_bare = _setup(tmp_path)
    _push_src(src, {"learning/a.py": "x\n", "pode/rl/p.txt": "y\n", "README.md": "b\n"}, "add")
    run = _run(dst, tmp_path, "--push")
    assert run.returncode == 0, run.stdout + run.stderr
    assert _tree(dst_bare) == {"README.md", "pc/fleet_core/test/test_ok.py", "pode/rl/p.txt"}
    assert _git(dst_bare, "show", "main:README.md") == "b"
    # 원본 커밋이 조상이라 다음 동기화는 합칠 것이 없다
    run = _run(dst, tmp_path, "--push")
    assert run.returncode == 0 and "already has" in run.stdout
    assert _git(dst, "status", "--porcelain") == ""  # 클론 작업 트리는 그대로다


def test_learning_edit_after_the_drop_does_not_block(tmp_path):
    src, dst, dst_bare = _setup(tmp_path)
    _push_src(src, {"learning/a.py": "x\n"}, "add learning")
    assert _run(dst, tmp_path, "--push").returncode == 0
    # 지운 뒤 원본에서 learning/을 고치면 수정-삭제 충돌이 나지만 지우는 쪽으로 풀린다
    _push_src(src, {"learning/a.py": "x2\n", "learning/new.py": "n\n", "README.md": "c\n"}, "edit")
    run = _run(dst, tmp_path, "--push")
    assert run.returncode == 0, run.stdout + run.stderr
    assert not any(name.startswith("learning/") for name in _tree(dst_bare))
    assert _git(dst_bare, "show", "main:README.md") == "c"


def test_conflict_outside_learning_stops_without_push(tmp_path):
    src, dst, dst_bare = _setup(tmp_path)
    _commit(dst, {"README.md": "nav2fleet only\n"}, "direct edit")
    _git(dst, "push", "-q", "origin", "main")
    before = _git(dst_bare, "rev-parse", "main")
    _push_src(src, {"README.md": "pode-drive\n"}, "edit readme")
    run = _run(dst, tmp_path, "--push")
    assert run.returncode == 1 and "README.md" in run.stdout
    assert _git(dst_bare, "rev-parse", "main") == before


def test_without_push_leaves_a_branch_and_main_untouched(tmp_path):
    src, dst, dst_bare = _setup(tmp_path)
    before = _git(dst_bare, "rev-parse", "main")
    _push_src(src, {"learning/a.py": "x\n", "b.txt": "b\n"}, "add")
    run = _run(dst, tmp_path)
    assert run.returncode == 0, run.stdout + run.stderr
    assert _git(dst_bare, "rev-parse", "main") == before
    assert set(_git(dst, "ls-tree", "-r", "--name-only", "sync/pode-drive").splitlines()) == {
        "README.md", "b.txt", "pc/fleet_core/test/test_ok.py"}


def test_failing_fleet_tests_block_the_push(tmp_path):
    src, dst, dst_bare = _setup(tmp_path)
    before = _git(dst_bare, "rev-parse", "main")
    _push_src(src, {"pc/fleet_core/test/test_ok.py": "def test_ok():\n    assert False\n"}, "break")
    run = _run(dst, tmp_path, "--push")
    assert run.returncode == 1 and "not pushed" in run.stdout
    assert _git(dst_bare, "rev-parse", "main") == before


def test_refuses_a_clone_of_another_repo(tmp_path):
    src, _, _ = _setup(tmp_path)
    run = _run(src, tmp_path, "--push")  # origin이 pode-drive다
    assert run.returncode == 1 and "not roscamp-nav2fleet" in run.stdout
    assert _run(src, tmp_path, "--force").returncode == 2


def _worktrees(repo):
    return [line for line in _git(repo, "worktree", "list", "--porcelain").splitlines()
            if line.startswith("worktree ")]


def test_relative_clone_path_leaves_no_worktree_behind(tmp_path):
    src, dst, dst_bare = _setup(tmp_path)
    _push_src(src, {"learning/a.py": "x\n", "b.txt": "b\n"}, "add")
    env = dict(ENV, SRC_URL=str(tmp_path / "roscamp-pode-drive.git"))
    run = subprocess.run(["bash", str(SCRIPT), "dst", "--push"], cwd=tmp_path, env=env,
                         capture_output=True, text=True, timeout=120)
    assert run.returncode == 0, run.stdout + run.stderr
    assert len(_worktrees(dst)) == 1
    # 충돌로 멈춰도 임시 worktree가 남지 않는다
    _git(dst, "pull", "-q", "--ff-only")
    _commit(dst, {"b.txt": "nav2fleet\n"}, "direct")
    _git(dst, "push", "-q", "origin", "HEAD:main")
    _push_src(src, {"b.txt": "pode-drive\n"}, "edit")
    run = subprocess.run(["bash", str(SCRIPT), "dst", "--push"], cwd=tmp_path, env=env,
                         capture_output=True, text=True, timeout=120)
    assert run.returncode == 1
    assert len(_worktrees(dst)) == 1


def test_main_that_already_has_the_source_with_learning_gets_it_dropped(tmp_path):
    src, dst, dst_bare = _setup(tmp_path)
    _push_src(src, {"learning/a.py": "x\n"}, "add")
    # 누가 nav2fleet main을 원본 그대로 옮겨 놓았다
    _git(src, "push", "-q", str(dst_bare), "main:main")
    run = _run(dst, tmp_path, "--push")
    assert run.returncode == 0, run.stdout + run.stderr
    assert not any(name.startswith("learning/") for name in _tree(dst_bare))
    run = _run(dst, tmp_path, "--push")
    assert run.returncode == 0 and "already has" in run.stdout


def test_refuses_a_push_url_of_another_repo(tmp_path):
    src, dst, dst_bare = _setup(tmp_path)
    _git(dst, "remote", "set-url", "--push", "origin", str(tmp_path / "roscamp-pode-drive.git"))
    _push_src(src, {"b.txt": "b\n"}, "add")
    run = _run(dst, tmp_path, "--push")
    assert run.returncode == 1 and "not roscamp-nav2fleet" in run.stdout
