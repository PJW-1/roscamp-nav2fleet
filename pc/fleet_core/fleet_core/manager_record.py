"""FleetManager의 기록과 입력 검사: 입력을 JSON 줄로 남기고, 잘못된 입력은 거른다.

재생 형식은 tools/replay.py, 검사 규칙은 common/validate.py.
"""
from __future__ import annotations

import json
from dataclasses import asdict, is_dataclass

from fleet_core.common.commands import EventOut, MoveCmd, OrderRecord
from fleet_core.common.validate import checked, robot_problem, text


def _recordable(value):
    """JSON에 못 쓰는 값은 repr로 남긴다. 기록 때문에 입력 처리가 멈추면 안 된다."""
    if is_dataclass(value) and not isinstance(value, type):
        return asdict(value)
    return repr(value)


def _recorded_orders(orders):
    """OrderRecord는 필드로, dict와 객체는 repr 문자열로 남긴다. 재생에서도 주문이 아니다."""
    if not isinstance(orders, (list, tuple)):
        return orders
    return [asdict(o) if isinstance(o, OrderRecord)
            else o if o is None or isinstance(o, (str, int, float, list))
            else repr(o) for o in orders]


class ManagerRecord:
    def check_recordable(self) -> None:
        """기록할 수 없으면 ValueError. 기록 파일을 열기 전에 부른다(옛 파일을 지우지 않게).

        재생은 지도를 첫 줄의 본문으로 다시 만든다(traffic/site.py의 site_from_text).
        """
        if not getattr(self.site, "source_text", ""):
            raise ValueError("a record needs the text of the map, and this map has none")

    def record_to(self, write) -> None:
        """모든 입력과 결과를 한 줄씩 JSON으로 `write`에 넘긴다.

        재생은 첫 줄에서 시작하므로 첫 입력보다 먼저 불러야 한다.
        """
        self.check_recordable()
        if self._fed:
            raise RuntimeError("a record has to start before the first input")
        self._write = write
        write(json.dumps({"recorded": True, "robots": self.traffic.robots,
                          "params": asdict(self.params), "map": self.site.source_text}))

    def _input(self, method, *args) -> None:
        self._fed = True
        self._time = self._clock()
        if self._write:
            self._write(self._input_line(method, args))

    def _input_line(self, method, args) -> str:
        """기록 줄. 못 쓰는 입력도 처리는 계속하고, 재생할 수 없다는 표시만 남긴다."""
        try:
            if method == "on_order_board":
                args = (args[0], _recorded_orders(args[1]))
            return json.dumps({"t": self._time, "in": method, "args": args}, default=_recordable)
        except Exception as error:  # 순환 참조, 문자열이 아닌 키, repr 실패
            return json.dumps({"t": self._time, "in": method,
                               "unrecordable": type(error).__name__})

    def _reject(self, detail, robot="") -> None:
        name = text(robot) if isinstance(robot, str) else ""
        known = name if name in self.traffic.robots else ""
        self._out.append(EventOut("WARN", "INPUT_REJECTED", detail, known))

    def _topic_rejected(self, source, problem, what, robot="") -> bool:
        """problem이 있으면 True. 출처마다 한 번 알리고, 정상 메시지가 오면 다시 무장한다."""
        if not problem:
            self._rejecting.discard(source)
            return False
        if source not in self._rejecting:
            self._rejecting.add(source)
            self._reject(f"{what}: {problem}", robot)
        return True

    def _robot_refused(self, what, robot) -> bool:
        """모르는 로봇의 goal 응답. 잘못 부른 경우에만 생긴다."""
        problem = checked(robot_problem, robot, self.traffic.robots)
        if problem:
            self._reject(f"{what} refused: {problem}")
        return bool(problem)

    def _note(self, kind, item) -> None:
        if self._write:
            fields = asdict(item)
            if isinstance(item, MoveCmd) and not item.path:
                del fields["path"]  # point 모드 줄은 path가 없던 기록과 같게 쓴다
            if isinstance(item, MoveCmd) and item.final_goal:
                del fields["final_goal"]  # 마지막 표시는 이 필드가 없던 기록과 같게 쓴다
            if isinstance(item, MoveCmd) and item.mode == "follow_path":
                del fields["mode"]  # 기본 방식은 이 필드가 없던 기록과 같게 쓴다
            self._write(json.dumps({"t": self._time, kind: type(item).__name__, **fields}))
