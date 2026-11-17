"""로봇의 goal은 goal 모듈(goal_life.py)의 입구로만 바꾼다.

`.goal =` 대입이 믹스인 여러 곳에 흩어지면 goal을 바꿀 때 함께 하던 일(멈춤 감시 초기화 등)을
빠뜨리기 쉽다. ast로 zone 패키지를 읽어 `.goal =` 대입이 goal_life.py와 ZoneBase.__init__에만
있는지 본다.
"""
import ast
from pathlib import Path

import fleet_core.traffic.zone as zone

ZONE = Path(zone.__file__).parent
ALLOWED = {("goal_life.py", None), ("base.py", "ZoneBase.__init__")}


def _goal_assignments(source: str, filename: str) -> list[tuple[str, str, int]]:
    found = []

    def targets(node):
        if isinstance(node, ast.Assign):
            return node.targets
        if isinstance(node, ast.AugAssign | ast.AnnAssign):
            return [node.target]
        return []

    def flat(target):
        if isinstance(target, ast.Tuple | ast.List):
            for item in target.elts:
                yield from flat(item)
        else:
            yield target

    def walk(node, where):
        for child in ast.iter_child_nodes(node):
            inner = where
            if isinstance(child, ast.ClassDef):
                inner = child.name
            elif isinstance(child, ast.FunctionDef | ast.AsyncFunctionDef):
                inner = f"{where}.{child.name}" if where else child.name
            for target in targets(child):
                for item in flat(target):
                    if isinstance(item, ast.Attribute) and item.attr == "goal":
                        found.append((filename, where, child.lineno))
            walk(child, inner)

    walk(ast.parse(source), "")
    return found


def _allowed(filename: str, where: str) -> bool:
    return (filename, None) in ALLOWED or (filename, where) in ALLOWED


def test_only_the_goal_module_assigns_a_goal():
    found = []
    for path in sorted(ZONE.glob("*.py")):
        found += _goal_assignments(path.read_text(encoding="utf-8"), path.name)
    assert [hit for hit in found if not _allowed(hit[0], hit[1])] == []
    assert any(name == "goal_life.py" for name, _, _ in found)


def test_the_check_can_fail():
    source = "class Drive:\n    def step(self, m):\n        m.goal, x = None, 1\n"
    assert _goal_assignments(source, "drive.py") == [("drive.py", "Drive.step", 3)]
    assert not _allowed("drive.py", "Drive.step")
