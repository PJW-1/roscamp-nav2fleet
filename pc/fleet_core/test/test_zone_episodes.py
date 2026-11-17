"""로봇의 감시 일화(Robot.watch의 각 Episode 묶음)는 정해 둔 모듈만 쓴다.

일화를 새 모듈이 건드리기 시작하면 그 일화를 누가 열고 닫는지가 다시 흩어진다. 늘리려면 이 표를
고치며 이유를 남긴다. `m.watch` 자체를 변수에 받아 돌려 쓰면 이 검사를 피하므로 그것도 막는다.
"""
import ast
from collections import defaultdict
from dataclasses import fields
from pathlib import Path

from fleet_core.traffic.zone.robots import Watches

ZONE = Path(__file__).resolve().parents[1] / "fleet_core" / "traffic" / "zone"

USERS = {
    "start": {"nostart.py"},
    "silent": {"silent.py"},
    "idle": {"silent.py"},
    "stall": {"goal_life.py", "watch.py"},
    "outside": {"watch.py"},
    "wait": {"base.py", "goals.py", "silent.py", "turns.py", "views.py"},
    "local_episode": {"blocked.py"},
    "redock": {"calls.py", "deadlock_watch.py", "drive.py", "redock.py"},
    "blocked": {"blocked.py", "yard_retreat.py"},
}


def _scan():
    users, loose = defaultdict(set), []
    for path in sorted(ZONE.glob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        parents = {id(child): node for node in ast.walk(tree)
                   for child in ast.iter_child_nodes(node)}
        for node in ast.walk(tree):
            if not isinstance(node, ast.Attribute) or node.attr != "watch":
                continue
            parent = parents.get(id(node))
            if isinstance(parent, ast.Attribute) and parent.value is node:
                users[parent.attr].add(path.name)
            elif not (isinstance(parent, ast.Call) and parent.func is node):
                loose.append(f"{path.name}:{node.lineno}")
    return users, loose


def test_every_episode_group_is_listed():
    assert set(USERS) == {f.name for f in fields(Watches)}


def test_each_episode_group_is_used_only_by_its_modules():
    users, _ = _scan()
    assert set(users) <= set(USERS), "a name that is no episode group"
    assert {name: users[name] for name in USERS} == USERS


def test_no_module_passes_the_whole_watch_record_around():
    _, loose = _scan()
    assert loose == []
