"""로봇의 goal은 goal 모듈(goal_life.py)의 입구로만 바꾼다: ast로 `.goal =` 대입이 goal_life.py와 ZoneBase.__init__에만 있는지 본다."""
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
        if isinstance(node, ast.AugAssign | ast.AnnAssign | ast.For | ast.AsyncFor
                      | ast.comprehension):
            return [node.target]
        if isinstance(node, ast.With | ast.AsyncWith):
            return [item.optional_vars for item in node.items if item.optional_vars is not None]
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == "setattr" and len(node.args) >= 2
                and isinstance(node.args[1], ast.Constant) and node.args[1].value == "goal"):
            return [ast.Attribute(node.args[0], "goal", ast.Store(), lineno=node.lineno)]
        return []

    def flat(target):
        if isinstance(target, ast.Tuple | ast.List):
            for item in target.elts:
                yield from flat(item)
        elif isinstance(target, ast.Starred):
            yield from flat(target.value)
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
                        found.append((filename, where, item.lineno))
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


def test_the_check_sees_every_way_to_assign():
    lines = [
        "def step(m, goals, place):",
        "    setattr(m, 'goal', None)",
        "    for m.goal in goals:",
        "        pass",
        "    *m.goal, x = goals",
        "    with place() as m.goal:",
        "        pass",
        "    [0 for m.goal in goals]",
        "    m.goal += 1",
        "    getattr(m, 'goal')",
        "    setattr(m, 'trip', None)",
    ]
    found = _goal_assignments("\n".join(lines) + "\n", "drive.py")
    assert sorted(line for _, _, line in found) == [2, 3, 5, 6, 8, 9]
