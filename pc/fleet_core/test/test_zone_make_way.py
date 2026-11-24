"""넓은 구역에서 서서 기다리는 로봇이 다른 로봇의 길을 막을 때 관제가 비키게 한다."""
from fleet_core.traffic.zone.goals import STANDING_WAIT_S
from fleet_core.traffic.zone.routes import Route
from zone_harness import DT, Harness

BODY_M = 0.12  # 두 로봇의 중심이 이보다 가까우면 몸이 닿는다


def run(world, seconds, body="pode2"):
    """body 로봇의 몸을 장애물로 두고 돌린다(시험 세계의 로봇은 서로를 보지 못한다)."""
    for _ in range(round(seconds / DT)):
        bot = world.bots[body]
        world.obstacles[:] = [(bot.x, bot.y, BODY_M)]
        world.step()


def run_until(world, done, timeout_s, body="pode2"):
    for _ in range(round(timeout_s / DT)):
        if done():
            return True
        run(world, DT, body)
    return done()


def queued_in_the_way():
    """pode3이 먼저 출고 도크로 가고, pode2는 대기 자리에서 0.14 m 비켜 선 채 차례를 기다린다."""
    world = Harness(starts={"pode1": "P-1", "pode2": "O_Q", "pode3": "P-3"})
    world.report()
    first = world.call("OUTBOUND", "pode3")
    world.now += DT / 2  # pode3이 먼저 불렸다
    second = world.call("OUTBOUND", "pode2")
    world.step()
    assert not world.moves("pode2"), "it waits at the queue spot where it stands"
    bot = world.bots["pode2"]
    bot.x, bot.y = 1.69, 0.55  # 관제의 도착 허용치(0.15 m) 안이지만 N_E로 가는 길 위다
    return world, first, second


def test_a_waiting_robot_that_stands_in_the_way_of_a_blocked_robot_moves_to_parking():
    world, first, second = queued_in_the_way()
    assert run_until(world, lambda: "ZONE_MARKED_BLOCKED" in world.codes("pode3"), 40.0)
    assert "HALL_YIELD" in world.codes("pode2"), "the robot in the way gives way"
    [away] = world.moves("pode2")
    # P-3이 더 가깝지만 막힌 pode3 쪽이다: 그 앞을 다시 지나지 않는 자리로 간다
    assert away.target_id == "P-2"
    assert run_until(world, lambda: world.result_of(first) is not None, 120.0)
    assert world.result_of(first).success, "the blocked robot reaches the dock"
    assert "DEADLOCK_SEEN" not in world.codes() and "GOAL_FAILED" not in world.codes()
    world.traffic.on_release("pode3")
    assert run_until(world, lambda: world.result_of(second) is not None, 300.0)
    assert world.result_of(second).success, "the robot that gave way still gets its turn"


def test_a_parked_robot_is_not_sent_away():
    world = Harness(starts={"pode1": "P-1", "pode2": "P-2", "pode3": "P-3"})
    world.report()
    world.call("WORK", "pode3")
    world.step()
    run(world, 40.0, body="pode2")
    assert not world.moves("pode2"), "a parked robot stays where it is"


def test_a_waiting_robot_away_from_the_blocked_way_stays():
    world = Harness(starts={"pode1": "N_E", "pode2": "O_Q", "pode3": "P-2"})
    world.report()
    world.call("OUTBOUND", "pode1")
    world.now += DT / 2
    world.call("OUTBOUND", "pode2")  # 도크가 빌 때까지 O_Q에서 기다린다
    world.call("WORK", "pode3")
    world.obstacles[:] = [(0.39, 0.40, BODY_M)]  # pode3이 P-2에서 N_W로 가는 길 위
    for _ in range(round(40.0 / DT)):
        world.step()
    assert "ZONE_MARKED_BLOCKED" in world.codes("pode3")
    assert "HALL_YIELD" not in world.codes() and not world.moves("pode2")


def test_a_parking_try_that_cannot_take_its_way_leaves_the_waiting_record_alone():
    world = Harness(layout="d", starts={"pode1": "P-1", "pode2": "O_Q", "pode3": "P-3"})
    world.report()
    traffic = world.traffic
    # 하나 남은 주차 자리 P-2는 서쪽에 있다. 건너갈 통로도, 급행으로 나가는 지점도 남이 쥐었다
    assert traffic.grants.force("pode1", ["CORR_M", "CORR_S"], [])
    assert traffic.grants.force("pode3", [], ["N_E"])
    assert not traffic._to_parking(traffic._robots["pode2"], "INFO", "HALL_YIELD", "goes to")
    assert traffic.grants.waiting() == {}, "it only tried: it does not wait for the corridor"
    traffic.grants.note_waiting("pode2", ("OUT_DOCK",))  # 이미 다른 것을 기다리던 로봇이면
    assert not traffic._to_parking(traffic._robots["pode2"], "INFO", "HALL_YIELD", "goes to")
    assert traffic.grants.waiting() == {"pode2": ("OUT_DOCK",)}, "what it waited for stays"


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
    """P-1에서 N_E로 가는 곧은 길이 P-2 주차 로봇의 몸에서 0.08 m를 지나면, 로봇이 돌아가지 못할 때(회피 흉내 none) 다른 지점을 거치게 한다."""
    world = Harness(starts={"pode1": "P-1", "pode2": "P-2", "pode3": "P-3"})
    world.report()
    call = world.call("OUTBOUND", "pode1")
    assert run_past(world, lambda: world.result_of(call) is not None, 300.0, "pode2", BODY_M)
    assert world.result_of(call).success
    assert "ZONE_MARKED_BLOCKED" in world.codes("pode1")
    waits = [e.detail for e in world.events if e.code == "ZONE_WAIT" and e.robot == "pode1"]
    assert not any("P-2" in detail for detail in waits), "it never waits for the parked spot"
    assert not world.moves("pode2"), "the parked robot stays"


def test_a_way_blocked_by_a_queued_robot_is_left_for_one_through_a_free_node():
    """P-3에서 N_E로 가는 곧은 길이 O_Q 대기 로봇의 몸에서 0.13 m를 지나고 비킬 주차 자리가 없으면 O_Q를 거치지 않는 길로 간다."""
    world = Harness(starts={"pode1": "P-2", "pode2": "O_Q", "pode3": "P-3"})
    world.report()
    first = world.call("OUTBOUND", "pode3")
    world.now += DT / 2  # pode3이 먼저 불렸다
    world.call("OUTBOUND", "pode2")
    assert run_past(world, lambda: world.result_of(first) is not None, 300.0, "pode2", 0.14)
    assert world.result_of(first).success
    waits = [e.detail for e in world.events if e.code == "ZONE_WAIT" and e.robot == "pode3"]
    assert not any("O_Q" in detail for detail in waits), "it never waits for the queue spot"


def test_the_wait_charged_for_a_node_depends_on_who_stands_there():
    """길 비용에 더하는 시간: 주차한 로봇의 지점 300 s, 대기 로봇의 지점은 찬 도크만큼, 나머지는 0."""
    world = Harness(starts={"pode1": "P-2", "pode2": "O_Q", "pode3": "P-3"})
    world.report()
    world.call("OUTBOUND", "pode3")
    world.now += DT / 2
    world.call("OUTBOUND", "pode2")
    world.step()
    traffic = world.traffic
    parked, queued, moving = (traffic._robots[name] for name in ("pode1", "pode2", "pode3"))
    assert queued.goal is None and moving.goal is not None
    passing = next(node for node in ("N_E", "X_E") if traffic.grants.node_holder(node) == "pode3")
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
    """배치 D 동쪽 마당: 먼저 불린 pode3의 곧은 길과 남쪽 통로 길이 모두 막히고 남은 길은 O_Q를 거친다."""
    world = Harness(layout="d", starts={"pode1": "P-2", "pode2": "O_Q", "pode3": "P-3"})
    world.report()
    first = world.call("OUTBOUND", "pode3")
    world.now += DT / 2  # pode3이 먼저 불렸다
    second = world.call("OUTBOUND", "pode2")
    return world, first, second


def test_a_queued_robot_that_cannot_be_passed_takes_the_dock_turn_first():
    """pode3은 O_Q가 비기를, pode2는 pode3의 차례를 기다려 둘 다 서면 앞을 막은 pode2가 먼저 도크에 간다."""
    world, first, second = stuck_behind_the_queue()
    assert run_among(world, lambda: world.result_of(second) is not None, 200.0, 0.2)
    assert world.result_of(second).success, "the robot in the way goes first"
    assert world.result_of(first) is None
    told = [e.detail for e in world.events if e.code == "DOCK_TURN_TAKEN"]
    assert told == ["goes before pode3, which waits for O_Q"]
    world.traffic.on_release("pode2")
    assert run_among(world, lambda: world.result_of(first) is not None, 300.0, 0.2)
    assert world.result_of(first).success, "the robot that was called first still gets there"
    assert "GOAL_FAILED" not in world.codes()
    assert world.problems == []


def test_the_dock_turn_stays_while_the_first_robot_can_still_move():
    """먼저 불린 로봇이 막힌 채 서 있는 동안(막힘 표시 전)에는 차례가 바뀌지 않는다."""
    world, first, second = stuck_behind_the_queue()
    assert run_among(world, lambda: "BLOCKED_WAIT" in world.codes("pode3"), 30.0, 0.2)
    run_among(world, lambda: False, 5.0, 0.2)
    assert "ZONE_MARKED_BLOCKED" not in world.codes(), "still before the mark"
    assert "DOCK_TURN_TAKEN" not in world.codes()
    assert not world.moves("pode2")


def test_a_robot_on_its_way_to_the_dock_keeps_the_turn():
    """차례를 앞지른 로봇의 도크 길이 설 수 있는 지점을 거치면 가는 중인 로봇이 차례를 지킨다."""
    world = Harness(layout="d", starts={"pode2": "P-1", "pode3": "P-3"})
    world.report()
    traffic = world.traffic
    for edge in (("P-3", "N_E"), ("P-3", "O_Q")):
        traffic._blocked[edge] = 1e9  # pode3은 남쪽 통로로 돌아간다
    going = world.call("OUTBOUND", "pode3")
    assert world.run_until(lambda: world.moves("pode3"), 5.0)
    route = traffic._robots["pode3"].route
    assert route.goal == "O" and route.granted < len(route.edges), "the dock zone comes later"
    assert route.turn, "the route carries the turn it was given"
    late = world.call("OUTBOUND", "pode2")
    traffic._robots["pode2"].asked_at = -1.0  # 먼저 불렸다가 앞지름을 당한 로봇인 셈이다
    while world.result_of(going) is None:
        world.step()
        other = traffic._robots["pode2"].route
        # order 모델은 대기 자리로 가는 길을 도크까지 잇는다: 차례(turn)를 받은 길만 도크로 떠난 것이다
        left = other is not None and other.goal == "O" and other.turn
        assert not left, "it does not leave for the dock as well"
        assert traffic.grants.node_holder("O") != "pode2"
        assert world.now < 200.0
    assert world.result_of(going).success and world.result_of(late) is None


def test_only_a_route_that_took_the_dock_turn_counts_as_on_its_way():
    """order 모델: 도크까지 이어진 대기 길도 차례(turn)를 받은 길이 아니면 줄에 선 것이라 먼저 불린 로봇의 차례를 가져가지 않는다."""
    world = Harness(layout="d", starts={"pode2": "O_Q", "pode3": "P-3"})
    world.report()
    traffic = world.traffic
    world.call("OUTBOUND", "pode3")
    world.now += DT / 2  # pode3이 먼저 불렸다
    world.call("OUTBOUND", "pode2")
    first, other = traffic._robots["pode3"], traffic._robots["pode2"]
    other.route = Route("O", traffic._way(other, "O"))
    assert traffic._is_next(first, first.dest)
    other.route.turn = True
    assert not traffic._is_next(first, first.dest)
    other.route.stale = True
    assert traffic._is_next(first, first.dest)
