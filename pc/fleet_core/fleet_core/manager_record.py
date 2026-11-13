from __future__ import annotations

import json
from dataclasses import asdict, is_dataclass

from fleet_core.common.commands import EventOut, MoveCmd, OrderRecord
from fleet_core.common.validate import checked, robot_problem, text


def _recordable(value):
    if is_dataclass(value) and not isinstance(value, type):
        return asdict(value)
    return repr(value)


def _recorded_orders(orders):
    if not isinstance(orders, (list, tuple)):
        return orders
    return [asdict(o) if isinstance(o, OrderRecord)
            else o if o is None or isinstance(o, (str, int, float, list))
            else repr(o) for o in orders]


class ManagerRecord:
    def check_recordable(self) -> None:
        if not getattr(self.site, "source_text", ""):
            raise ValueError("a record needs the text of the map, and this map has none")

    def record_to(self, write) -> None:
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
        try:
            if method == "on_order_board":
                args = (args[0], _recorded_orders(args[1]))
            return json.dumps({"t": self._time, "in": method, "args": args}, default=_recordable)
        except Exception as error:
            return json.dumps({"t": self._time, "in": method,
                               "unrecordable": type(error).__name__})

    def _reject(self, detail, robot="") -> None:
        name = text(robot) if isinstance(robot, str) else ""
        known = name if name in self.traffic.robots else ""
        self._out.append(EventOut("WARN", "INPUT_REJECTED", detail, known))

    def _topic_rejected(self, source, problem, what, robot="") -> bool:
        if not problem:
            self._rejecting.discard(source)
            return False
        if source not in self._rejecting:
            self._rejecting.add(source)
            self._reject(f"{what}: {problem}", robot)
        return True

    def _robot_refused(self, what, robot) -> bool:
        problem = checked(robot_problem, robot, self.traffic.robots)
        if problem:
            self._reject(f"{what} refused: {problem}")
        return bool(problem)

    def _note(self, kind, item) -> None:
        if self._write:
            fields = asdict(item)
            if isinstance(item, MoveCmd) and not item.path:
                del fields["path"]
            if isinstance(item, MoveCmd) and item.final_goal:
                del fields["final_goal"]
            if isinstance(item, MoveCmd) and item.mode == "follow_path":
                del fields["mode"]
            self._write(json.dumps({"t": self._time, kind: type(item).__name__, **fields}))
