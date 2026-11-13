from __future__ import annotations


def find_cycle(graph: dict[str, set[str]]) -> list[str] | None:
    done: set[str] = set()

    def walk(robot: str, trail: list[str]) -> list[str] | None:
        if robot in trail:
            return trail[trail.index(robot):]
        if robot in done:
            return None
        for holder in sorted(graph.get(robot, ())):
            cycle = walk(holder, [*trail, robot])
            if cycle:
                return cycle
        done.add(robot)
        return None

    for start in sorted(graph):
        cycle = walk(start, [])
        if cycle:
            return cycle
    return None


def priority(params, at_station: bool, carrying: bool) -> int:
    score = 0
    if at_station:
        score += params.priority_at_station
    if carrying:
        score += params.priority_has_cargo
    return score


def choose_retreat(cycle: list[str], priority_fn) -> str:
    return min(cycle, key=lambda robot: (priority_fn(robot), robot))
