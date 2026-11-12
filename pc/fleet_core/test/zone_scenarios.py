"""구역 교통 시험이 함께 쓰는 판(정해진 순서의 호출과 장애물). 시험 전용이다."""
from __future__ import annotations

from zone_harness import Harness


def serve_three(world: Harness) -> bool:
    """세 대가 채우기 호출 셋을 받고, 출고 도크를 거쳐, 주차 자리로 돌아간다."""
    world.step()
    calls = [world.call("WORK") for _ in range(3)]
    served: set[str] = set()

    def done() -> bool:
        for result in world.results:
            if result.goal_id in calls and result.success and result.robot_id not in served:
                served.add(result.robot_id)
                world.call("OUTBOUND", result.robot_id)
        for robot in world.bots:
            view = world.view(robot)
            if view["state"] == "AT_STATION" and view["destination"] == "OUTBOUND":
                world.traffic.on_release(robot)
        return len(served) == 3 and all(world.phase(r) == "PARKED" for r in world.bots)

    return world.run_until(done, timeout_s=600.0)


def blocked_express(world: Harness) -> bool:
    """작업 도크에서 출고 도크로 가는 길에 급행 한가운데가 막혀 있다."""
    world.report()
    call = world.call("OUTBOUND", "pinky1")
    world.obstacles.append((1.0, 0.89, 0.15))
    return world.run_until(lambda: world.result_of(call) is not None, 200.0)


def busy_outbound(world: Harness, hold_s: float = 40.0) -> bool:
    """출고 도크를 한 대가 오래 쓰는 동안 다른 한 대가 출고 도크로 불린다."""
    world.report()
    first = world.call("OUTBOUND", "pinky2")
    assert world.run_until(lambda: world.result_of(first) is not None, 30.0)
    call = world.call("OUTBOUND", "pinky1")
    world.run(hold_s)
    world.traffic.on_release("pinky2")
    return world.run_until(lambda: world.result_of(call) is not None, 200.0)


SCENARIOS = {
    "serve_three": (serve_three, None),
    "blocked_express": (blocked_express, {"pinky1": "W", "pinky2": "P-2", "pinky3": "P-3"}),
    "busy_outbound": (busy_outbound, {"pinky1": "W", "pinky2": "O", "pinky3": "P-3"}),
}


def play(name: str, params=None) -> Harness:
    scenario, starts = SCENARIOS[name]
    world = Harness(starts=starts, params=params)
    assert scenario(world), f"{name} did not finish:\n" + "\n".join(world.log[-30:])
    return world
