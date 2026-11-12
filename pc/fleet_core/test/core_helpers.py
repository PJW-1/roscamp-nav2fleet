"""테스트용 보조 함수: 틱 출력을 종류별로 거른다."""


def cmds(out, name):
    """틱 출력을 클래스 이름으로 거른다."""
    return [c for c in out if type(c).__name__ == name]


def codes(out):
    return [e.code for e in cmds(out, "EventOut")]
