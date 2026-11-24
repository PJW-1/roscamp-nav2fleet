"""조용히 멈춤(SILENT_STALL): 갈 곳이 있는 로봇이 나아가지 않는데 관제가 아무 말 없이 silent_limit_s가 지나면 한 번 알린다."""
from dataclasses import asdict, replace

import zone_harness
from fleet_core.common.params import load_params
from zone_harness import Harness

LIMIT = load_params().silent_limit_s


def count(world, code, robot=""):
    return world.codes(robot).count(code)


def quiet_goal(**kwargs):
    """pode2가 goal을 받고 서 있다. 제자리 멈춤 감시를 꺼서 관제의 결함을 흉내 낸다."""
    world = Harness(speed=0.0, **kwargs)
    world.traffic._watch_stall = lambda m, now: None
    world.traffic._start_watch.watch = lambda m, now, running: None
    world.step()
    world.call("WORK", "pode2")
    world.step()
    assert world.moves("pode2")
    return world


def test_the_shipped_limit_is_60_s_and_records_without_it_run_without_the_watch():
    assert LIMIT == 60.0
    fields = asdict(load_params())
    del fields["silent_limit_s"]
    assert type(load_params())(**fields).silent_limit_s == 0.0


def test_a_robot_with_an_open_goal_that_stands_while_the_fleet_says_nothing_is_reported_once():
    world = quiet_goal()
    world.run(LIMIT - 1.0)
    assert count(world, "SILENT_STALL") == 0
    world.run(1.5)
    found = [(e.level, e.robot) for e in world.events if e.code == "SILENT_STALL"]
    assert found == [("WARN", "pode2")]
    detail = next(e.detail for e in world.events if e.code == "SILENT_STALL")
    assert detail.startswith("TO_WORK at P-2: no progress and no fleet event for 60 s")
    world.run(2 * LIMIT)
    assert count(world, "SILENT_STALL") == 1, "once per episode"


def test_an_event_about_the_robot_starts_a_new_episode():
    world = quiet_goal()
    world.run(LIMIT + 1.0)
    assert count(world, "SILENT_STALL") == 1
    world.traffic._event("INFO", "SOMETHING", "the fleet speaks", "pode2")
    world.run(LIMIT - 1.0)
    assert count(world, "SILENT_STALL") == 1
    world.run(1.5)
    assert count(world, "SILENT_STALL") == 2


def test_an_event_about_another_robot_does_not_count():
    world = quiet_goal()
    world.run(LIMIT / 2)
    world.traffic._event("INFO", "SOMETHING", "about someone else", "pode1")
    world.run(LIMIT / 2 + 0.5)
    assert count(world, "SILENT_STALL", "pode2") == 1


def test_moving_starts_a_new_episode():
    world = quiet_goal()
    world.run(LIMIT - 5.0)
    world.speed = 0.12
    world.run(1.0)  # 0.12 m 나아간다
    world.speed = 0.0
    world.run(LIMIT - 1.0)
    assert count(world, "SILENT_STALL") == 0
    world.run(1.5)
    assert count(world, "SILENT_STALL") == 1


def test_the_watch_is_off_at_zero():
    world = quiet_goal(params=replace(load_params(), silent_limit_s=0.0))
    world.run(2 * LIMIT)
    assert count(world, "SILENT_STALL") == 0


def test_a_robot_with_no_goal_and_nothing_to_wait_for_is_told_about_by_robot_idle():
    """ROBOT_IDLE이 idle_watch_s마다 말하므로 조용하지 않다. 그 감시를 끄면 이 감시가 잡는다."""
    for idle_s, expected in ((load_params().idle_watch_s, 0), (0.0, 1)):
        world = Harness(params=replace(load_params(), idle_watch_s=idle_s))
        world.step()
        world.traffic._choose = lambda m: None  # 길도 기다릴 것도 없다
        world.call("WORK", "pode2")
        world.run(2 * LIMIT)
        assert count(world, "SILENT_STALL") == expected, idle_s


# ---- 관제가 알고 세워 둔 로봇 ----

def test_a_robot_parked_with_nothing_to_do_is_not_silent():
    world = Harness()
    world.run(3 * LIMIT)
    assert count(world, "SILENT_STALL") == 0


def test_a_robot_waiting_for_a_busy_dock_is_not_silent():
    world = Harness(starts={"pode1": "W", "pode2": "P-2", "pode3": "W_Q"})
    world.report()
    world.call("WORK", "pode1")
    world.call("WORK", "pode3")
    world.call("WORK", "pode2")
    world.run(3 * LIMIT)
    assert "DOCK_QUEUED" in world.codes("pode2")
    assert count(world, "SILENT_STALL") == 0


def test_a_robot_served_at_its_station_is_not_silent():
    world = Harness(starts={"pode1": "W", "pode2": "P-2", "pode3": "P-3"})
    world.report()
    call = world.call("WORK", "pode1")
    assert world.run_until(lambda: world.result_of(call) is not None, 30.0)
    assert world.phase("pode1") == "AT_STATION"
    world.run(3 * LIMIT)
    assert count(world, "SILENT_STALL") == 0


def test_a_robot_that_fails_or_goes_silent_is_not_watched():
    world = quiet_goal()
    world.traffic.on_robot_fault("pode2", "MOTOR")
    world.run(2 * LIMIT)
    assert count(world, "SILENT_STALL") == 0
    world = quiet_goal()
    world.silent.add("pode2")
    world.run(2 * LIMIT)
    assert "COMM_LOST" in world.codes("pode2") and count(world, "SILENT_STALL") == 0


def test_a_robot_that_never_reports_is_not_watched(monkeypatch):
    world = Harness()
    world.silent.add("pode2")
    world.step()
    world.call("WORK", "pode2")
    world.run(2 * LIMIT)
    assert count(world, "SILENT_STALL") == 0


def test_a_robot_held_back_for_a_retry_or_a_yield_is_not_silent(monkeypatch):
    world = quiet_goal()
    m = world.traffic._robots["pode2"]
    world.run(LIMIT - 5.0)
    m.avoid.yield_until = world.now + 3.0
    world.run(LIMIT - 1.0)
    assert count(world, "SILENT_STALL") == 0, "the yield ended an episode"
    m.retry_at = world.now + 3.0
    world.run(LIMIT - 1.0)
    assert count(world, "SILENT_STALL") == 0, "so did the retry wait"


def test_a_docking_robot_counts_as_one_with_somewhere_to_go(monkeypatch):
    monkeypatch.setattr(zone_harness, "DOCK_S", 1000.0)  # 도킹이 끝나지 않는다
    world = Harness(starts={"pode1": "W", "pode2": "P-2", "pode3": "P-3"})
    world.traffic._watch_stall = lambda m, now: None
    world.report()
    world.call("WORK", "pode1")
    world.run(LIMIT + 1.0)
    assert count(world, "SILENT_STALL", "pode1") == 1
