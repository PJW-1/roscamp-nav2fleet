"""넓은 구역에서 서서 기다리는 로봇이 다른 로봇의 길을 몸으로 막는 경우.

Gazebo 3대 경기에서 나왔다: 출고 대기 자리로 가던 로봇이 진입 지점(N_E) 바로 앞에 섰고, 먼저
도크로 가던 로봇은 N_E를 쥔 채 거기 닿지 못했다. 구역 대기 그래프에는 순환이 없어서 관제는
아무것도 하지 않았고, 막힌 로봇이 세 번 실패한 뒤 격리됐다.
"""
from fleet_core.common.commands import MoveCmd
from zone_harness import DT, Harness

BODY_M = 0.12  # 두 로봇의 중심이 이보다 가까우면 몸이 닿는다


def run(world, seconds, body="pinky2"):
    """body 로봇의 몸을 장애물로 두고 돌린다(시험 세계의 로봇은 서로를 보지 못한다)."""
    for _ in range(round(seconds / DT)):
        bot = world.bots[body]
        world.obstacles[:] = [(bot.x, bot.y, BODY_M)]
        world.step()


def run_until(world, done, timeout_s, body="pinky2"):
    for _ in range(round(timeout_s / DT)):
        if done():
            return True
        run(world, DT, body)
    return done()


def queued_in_the_way():
    """pinky3이 먼저 출고 도크로 가고, pinky2는 대기 자리에서 0.14 m 비켜 선 채 차례를 기다린다."""
    world = Harness(starts={"pinky1": "P-1", "pinky2": "O_Q", "pinky3": "P-3"})
    world.report()
    first = world.call("OUTBOUND", "pinky3")
    world.now += DT / 2  # pinky3이 먼저 불렸다
    second = world.call("OUTBOUND", "pinky2")
    world.step()
    assert not world.sent(MoveCmd, "pinky2"), "it waits at the queue spot where it stands"
    bot = world.bots["pinky2"]
    bot.x, bot.y = 1.64, 0.55  # 관제의 도착 허용치(0.15 m) 안이지만 N_E로 가는 길 위다
    return world, first, second


def test_a_waiting_robot_that_stands_in_the_way_of_a_blocked_robot_moves_to_parking():
    world, first, second = queued_in_the_way()
    assert run_until(world, lambda: "ZONE_MARKED_BLOCKED" in world.codes("pinky3"), 40.0)
    assert "HALL_YIELD" in world.codes("pinky2"), "the robot in the way gives way"
    [away] = world.sent(MoveCmd, "pinky2")
    # P-3이 더 가깝지만 막힌 pinky3 쪽이다: 그 앞을 다시 지나지 않는 자리로 간다
    assert away.target_id == "P-2"
    assert run_until(world, lambda: world.result_of(first) is not None, 120.0)
    assert world.result_of(first).success, "the blocked robot reaches the dock"
    assert "DEADLOCK_SEEN" not in world.codes() and "GOAL_FAILED" not in world.codes()
    world.traffic.on_release("pinky3")
    assert run_until(world, lambda: world.result_of(second) is not None, 300.0)
    assert world.result_of(second).success, "the robot that gave way still gets its turn"


def test_a_parked_robot_is_not_sent_away():
    world = Harness(starts={"pinky1": "P-1", "pinky2": "P-2", "pinky3": "P-3"})
    world.report()
    world.call("WORK", "pinky3")
    world.step()
    run(world, 40.0, body="pinky2")
    assert not world.sent(MoveCmd, "pinky2"), "a parked robot stays where it is"


def test_a_waiting_robot_away_from_the_blocked_way_stays():
    world = Harness(starts={"pinky1": "N_E", "pinky2": "O_Q", "pinky3": "P-2"})
    world.report()
    world.call("OUTBOUND", "pinky1")
    world.now += DT / 2
    world.call("OUTBOUND", "pinky2")  # 도크가 빌 때까지 O_Q에서 기다린다
    world.call("WORK", "pinky3")
    world.obstacles[:] = [(0.39, 0.40, BODY_M)]  # pinky3이 P-2에서 N_W로 가는 길 위
    for _ in range(round(40.0 / DT)):
        world.step()
    assert "ZONE_MARKED_BLOCKED" in world.codes("pinky3")
    assert "HALL_YIELD" not in world.codes() and not world.sent(MoveCmd, "pinky2")


def test_a_parking_try_that_cannot_take_its_way_leaves_the_waiting_record_alone():
    world = Harness(layout="d", starts={"pinky1": "P-1", "pinky2": "O_Q", "pinky3": "P-3"})
    world.report()
    traffic = world.traffic
    # 하나 남은 주차 자리 P-2는 서쪽에 있다. 건너갈 통로도, 급행으로 나가는 지점도 남이 쥐었다
    assert traffic.grants.force("pinky1", ["CORR_M", "CORR_S"], [])
    assert traffic.grants.force("pinky3", [], ["N_E"])
    assert not traffic._to_parking(traffic._robots["pinky2"], "INFO", "HALL_YIELD", "goes to")
    assert traffic.grants.waiting() == {}, "it only tried: it does not wait for the corridor"
    traffic.grants.note_waiting("pinky2", ("OUT_DOCK",))  # 이미 다른 것을 기다리던 로봇이면
    assert not traffic._to_parking(traffic._robots["pinky2"], "INFO", "HALL_YIELD", "goes to")
    assert traffic.grants.waiting() == {"pinky2": ("OUT_DOCK",)}, "what it waited for stays"
