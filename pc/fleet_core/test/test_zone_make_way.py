"""넓은 구역에서 서서 기다리는 로봇이 다른 로봇의 길을 몸으로 막는 경우.

Gazebo 3대 경기에서 나왔다: 출고 대기 자리로 가던 로봇이 진입 지점(N_E) 바로 앞에 섰고, 먼저
도크로 가던 로봇은 N_E를 쥔 채 거기 닿지 못했다. 구역 대기 그래프에는 순환이 없어서 관제는
아무것도 하지 않았고, 막힌 로봇이 세 번 실패한 뒤 격리됐다.
"""
from fleet_core.common.commands import MoveCmd
from fleet_core.traffic.zone.goals import STANDING_WAIT_S
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


# ---- 선 로봇이 쥔 지점을 거치는 길은 고르지 않는다(회피 흉내 none에서 끝나지 않던 두 경우) ----

def run_past(world, done, timeout_s, body, radius):
    """body 로봇의 몸(반지름 radius)을 장애물로 두고 done이 될 때까지 돌린다."""
    for _ in range(round(timeout_s / DT)):
        if done():
            return True
        bot = world.bots[body]
        world.obstacles[:] = [(bot.x, bot.y, radius)]
        world.step()
    return done()


def test_a_way_blocked_by_a_parked_robot_is_left_for_one_through_a_free_node():
    """P-1에서 N_E로 가는 곧은 길은 P-2에 주차한 로봇의 몸에서 0.08 m를 지난다. 로봇이 돌아가지
    못하면(회피 흉내 none) 관제가 다른 지점을 거치게 해야 한다. P-2는 그 로봇이 쥐고 있다."""
    world = Harness(starts={"pinky1": "P-1", "pinky2": "P-2", "pinky3": "P-3"})
    world.report()
    call = world.call("OUTBOUND", "pinky1")
    assert run_past(world, lambda: world.result_of(call) is not None, 300.0, "pinky2", BODY_M)
    assert world.result_of(call).success
    assert "ZONE_MARKED_BLOCKED" in world.codes("pinky1")
    waits = [e.detail for e in world.events if e.code == "ZONE_WAIT" and e.robot == "pinky1"]
    assert not any("P-2" in detail for detail in waits), "it never waits for the parked spot"
    assert not world.sent(MoveCmd, "pinky2"), "the parked robot stays"


def test_a_way_blocked_by_a_queued_robot_is_left_for_one_through_a_free_node():
    """P-3에서 N_E로 가는 곧은 길은 O_Q에서 차례를 기다리는 로봇의 몸에서 0.13 m를 지난다. 비킬
    주차 자리가 없으면(P-2는 남이 섰고 P-1은 막힌 로봇 쪽이다) O_Q를 거치지 않는 길로 간다."""
    world = Harness(starts={"pinky1": "P-2", "pinky2": "O_Q", "pinky3": "P-3"})
    world.report()
    first = world.call("OUTBOUND", "pinky3")
    world.now += DT / 2  # pinky3이 먼저 불렸다
    world.call("OUTBOUND", "pinky2")
    assert run_past(world, lambda: world.result_of(first) is not None, 300.0, "pinky2", 0.14)
    assert world.result_of(first).success
    waits = [e.detail for e in world.events if e.code == "ZONE_WAIT" and e.robot == "pinky3"]
    assert not any("O_Q" in detail for detail in waits), "it never waits for the queue spot"


def test_the_wait_charged_for_a_node_depends_on_who_stands_there():
    """길 비용에 더하는 시간: 주차한 로봇의 지점 300 s, 차례를 기다리는 로봇의 지점은 찬 도크만큼,
    빈 지점과 제가 쥔 지점, 지나가는 로봇이 쥔 지점, 도크 지점은 0."""
    world = Harness(starts={"pinky1": "P-2", "pinky2": "O_Q", "pinky3": "P-3"})
    world.report()
    world.call("OUTBOUND", "pinky3")
    world.now += DT / 2
    world.call("OUTBOUND", "pinky2")
    world.step()
    traffic = world.traffic
    parked, queued, moving = (traffic._robots[name] for name in ("pinky1", "pinky2", "pinky3"))
    assert queued.goal is None and moving.goal is not None
    passing = next(node for node in ("N_E", "X_E") if traffic.grants.node_holder(node) == "pinky3")
    wait = traffic._node_wait
    assert wait(moving, "P-2") == STANDING_WAIT_S == 300.0
    assert wait(moving, "O_Q") == world.params.dock_wait_estimate_s
    assert wait(moving, "P-1") == 0.0, "nobody there"
    assert wait(moving, passing) == 0.0, "its own node"
    assert wait(parked, passing) == 0.0, "the holder is passing through"
    assert wait(queued, "O_Q") == 0.0 and wait(parked, "O") == 0.0


# ---- 도크 차례: 먼저 불린 로봇이 뒤 차례 로봇이 선 지점을 기다리면 뒤 차례가 먼저 간다 ----

def run_among(world, done, timeout_s, radius):
    """서 있는 로봇들의 몸(반지름 radius)을 장애물로 두고 done이 될 때까지 돌린다."""
    for _ in range(round(timeout_s / DT)):
        if done():
            return True
        world.obstacles[:] = [(bot.x, bot.y, radius) for bot in world.bots.values()
                              if bot.cmd is None and bot.dock is None]
        world.step()
    return done()


def stuck_behind_the_queue():
    """배치 D의 동쪽 마당. 먼저 불린 pinky3(P-3)의 곧은 길은 O_Q에서 차례를 기다리는 pinky2의 몸에
    막히고, 남쪽 통로로 도는 길은 P-2에 주차한 pinky1의 몸에 막힌다. 남은 길은 O_Q를 거친다."""
    world = Harness(layout="d", starts={"pinky1": "P-2", "pinky2": "O_Q", "pinky3": "P-3"})
    world.report()
    first = world.call("OUTBOUND", "pinky3")
    world.now += DT / 2  # pinky3이 먼저 불렸다
    second = world.call("OUTBOUND", "pinky2")
    return world, first, second


def test_a_queued_robot_that_cannot_be_passed_takes_the_dock_turn_first():
    """pinky3은 O_Q가 비기를, pinky2는 pinky3의 차례를 기다리면 둘 다 서 있게 된다. 그때는 앞을
    막은 pinky2가 먼저 도크에 가고, 먼저 불린 pinky3은 그 뒤에 간다."""
    world, first, second = stuck_behind_the_queue()
    assert run_among(world, lambda: world.result_of(second) is not None, 200.0, 0.2)
    assert world.result_of(second).success, "the robot in the way goes first"
    assert world.result_of(first) is None
    told = [e.detail for e in world.events if e.code == "DOCK_TURN_TAKEN"]
    assert told == ["goes before pinky3, which waits for O_Q"]
    world.traffic.on_release("pinky2")
    assert run_among(world, lambda: world.result_of(first) is not None, 300.0, 0.2)
    assert world.result_of(first).success, "the robot that was called first still gets there"
    assert "GOAL_FAILED" not in world.codes()
    assert world.problems == []


def test_the_dock_turn_stays_while_the_first_robot_can_still_move():
    """먼저 불린 로봇이 막힌 채 서 있는 동안(막힘 표시 전)에는 차례가 바뀌지 않는다."""
    world, first, second = stuck_behind_the_queue()
    assert run_among(world, lambda: "BLOCKED_WAIT" in world.codes("pinky3"), 30.0, 0.2)
    run_among(world, lambda: False, 5.0, 0.2)
    assert "ZONE_MARKED_BLOCKED" not in world.codes(), "still before the mark"
    assert "DOCK_TURN_TAKEN" not in world.codes()
    assert not world.sent(MoveCmd, "pinky2")


def test_a_robot_on_its_way_to_the_dock_keeps_the_turn():
    """차례를 앞지른 로봇의 도크 길이 설 수 있는 지점을 거치면 도크 구역은 그 지점을 지나서 잡는다.
    그 사이에 먼저 불린 로봇도 차례라고 보면 둘이 같은 도크로 떠난다: 가는 중인 로봇이 차례를
    지킨다."""
    world = Harness(layout="d", starts={"pinky2": "P-1", "pinky3": "P-3"})
    world.report()
    traffic = world.traffic
    for link in (("P-3", "N_E"), ("P-3", "O_Q")):
        traffic._blocked[link] = 1e9  # pinky3은 남쪽 통로로 돌아간다
    going = world.call("OUTBOUND", "pinky3")
    assert world.run_until(lambda: world.sent(MoveCmd, "pinky3"), 5.0)
    trip = traffic._robots["pinky3"].trip
    assert trip.goal == "O" and trip.granted < len(trip.links), "the dock zone comes later"
    late = world.call("OUTBOUND", "pinky2")
    traffic._robots["pinky2"].asked_at = -1.0  # 먼저 불렸다가 앞지름을 당한 로봇인 셈이다
    world.run(3.0)
    other = traffic._robots["pinky2"].trip
    assert other is None or other.goal != "O", "it does not leave for the dock as well"
    assert world.run_until(lambda: world.result_of(going) is not None, 200.0)
    assert world.result_of(going).success and world.result_of(late) is None
