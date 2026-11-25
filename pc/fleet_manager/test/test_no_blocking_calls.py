"""fleet_manager 패키지에 동기 서비스 호출과 블로킹 스핀이 없는지 소스를 정적으로 검사한다."""
import ast
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1] / "fleet_manager"
BANNED_NAMES = {"spin_until_future_complete", "spin_once"}


def find_blocking_calls(source, filename="<string>"):
    """.call(, spin_until_future_complete, spin_once 호출 위치를 (파일:줄, 이름)으로 모은다."""
    found = []
    for node in ast.walk(ast.parse(source, filename)):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        name = func.attr if isinstance(func, ast.Attribute) else getattr(func, "id", None)
        if (name == "call" and isinstance(func, ast.Attribute)) or name in BANNED_NAMES:
            found.append((f"{filename}:{node.lineno}", name))
    return found


def test_the_checker_finds_a_synchronous_call():
    """검사기가 실제로 실패할 수 있음을 작은 예제로 보인다."""
    code = "def f(client, req):\n    return client.call(req)\n"
    assert find_blocking_calls(code, "x.py") == [("x.py:2", "call")]
    assert find_blocking_calls("rclpy.spin_once(node)\nspin_until_future_complete(n, f)\n")


def test_the_package_has_no_blocking_calls():
    """패키지 안에 동기 호출이 없다."""
    files = sorted(PACKAGE.rglob("*.py"))
    assert files
    found = []
    for path in files:
        found += find_blocking_calls(path.read_text(encoding="utf-8"), str(path))
    assert not found, "동기 호출 발견: " + ", ".join(f"{loc} {name}" for loc, name in found)
