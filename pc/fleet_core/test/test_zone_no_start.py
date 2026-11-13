"""움직이기 시작하지 않음(NO_START): 관제가 이동이나 후진 goal을 보낸 뒤 start_limit_s 안에 보고
위치가 2 cm 넘게 바뀌지 않았고 goal이 아직 열려 있으면 그 goal에 한 번 알린다."""
from dataclasses import asdict, replace

import zone_harness
from fleet_core.common.commands import DockCmd
from fleet_core.common.params import load_params
from fleet_core.fleet_map import NAV_GOAL
from fleet_core.traffic.mission import MOVE
from fleet_core.traffic.zone.robots import Goal
from zone_harness import Harness

LIMIT = load_params().start_limit_s


def count(world, code, robot=""):
    return world.codes(robot).count(code)


def called(speed=0.0, **kwargs):
    world = Harness(speed=speed, **kwargs)
    world.step()
    world.call("WORK", "pinky2")
    world.step()
    assert world.moves("pinky2")
    return world


def reports(world, robot, state):
    plain = Harness._state.__get__(world)
    world._state = lambda bot: state if bot.name == robot else plain(bot)


def test_the_shipped_limit_is_5_s_and_records_without_it_run_without_the_watch():
    assert LIMIT == 5.0
    fields = asdict(load_params())
    del fields["start_limit_s"]
    assert type(load_params())(**fields).start_limit_s == 0.0


def test_a_goal_the_robot_does_not_start_on_is_reported_once():
    world = called()
    goal = world.moves("pinky2")[0].goal_id
    end = world.legs("pinky2")[-1].target_id  # order 모델: base 끝(W), 끄면 첫 구간 끝(N_W)
    world.run(LIMIT - 0.5)
    assert count(world, "NO_START") == 0
    world.run(0.7)
    found = [(e.level, e.robot, e.detail) for e in world.events if e.code == "NO_START"]
    assert found == [("WARN", "pinky2", f"move goal {goal} to {end}: moved less than 0.02 m "
                                        "in 5 s")]
    world.run(0.5)
    assert count(world, "NO_START") == 1, "once per goal"


def test_each_new_goal_is_watched_on_its_own():
    world = called()
    assert world.run_until(lambda: len(world.moves("pinky2")) == 2, 20.0)  # 다시 낸 goal
    world.run(LIMIT + 0.5)
    goals = [c.goal_id for c in world.moves("pinky2")][:2]
    details = [e.detail for e in world.events if e.code == "NO_START"]
    assert [d.split()[2] for d in details[:2]] == [str(g) for g in goals]


def test_a_robot_that_starts_within_the_limit_is_not_reported():
    world = called(speed=0.12)
    world.run(3 * LIMIT)
    assert count(world, "NO_START") == 0


def test_moving_2_cm_or_less_is_not_a_start():
    world = called(speed=0.015 / LIMIT)  # 5 s에 1.5 cm
    world.run(LIMIT + 0.5)
    assert count(world, "NO_START") == 1


def test_a_robot_that_reports_blocked_or_recovering_from_the_start_is_not_reported():
    for state in ("BLOCKED", "RECOVERING"):
        world = Harness(speed=0.0)
        reports(world, "pinky2", state)
        world.step()
        world.call("WORK", "pinky2")
        world.run(3 * LIMIT)
        assert count(world, "NO_START") == 0, state


def test_docking_is_not_watched(monkeypatch):
    monkeypatch.setattr(zone_harness, "DOCK_S", 1000.0)  # 도킹이 끝나지 않는다
    world = Harness(starts={"pinky1": "W", "pinky2": "P-2", "pinky3": "P-3"})
    world.report()
    world.call("WORK", "pinky1")
    world.run(3 * LIMIT)
    assert world.sent(DockCmd, "pinky1") and count(world, "NO_START") == 0


def test_a_retreat_the_robot_does_not_start_on_is_reported():
    world = Harness(starts={"pinky1": "W", "pinky2": "P-2", "pinky3": "P-3"})
    world.report()
    call = world.call("WORK", "pinky1")
    assert world.run_until(lambda: world.result_of(call) is not None, 30.0)
    plain = Harness._move.__get__(world)
    world._move = lambda bot: None if bot.name == "pinky1" else plain(bot)  # 바퀴가 돌지 않는다
    world.call("OUTBOUND", "pinky1")
    world.step()
    [retreat] = [leg for leg in world.legs("pinky1") if leg.mode == "retreat"]
    world.run(LIMIT + 0.5)
    details = [e.detail for e in world.events if e.code == "NO_START"]
    # order 모델에서 물러나기는 O로 가는 order의 첫 조각이다. 알림은 order goal과 base 끝을 말한다
    assert details == [f"move goal {retreat.goal_id} to O: moved less than 0.02 m in 5 s"]


def test_a_goal_to_where_the_robot_already_stands_is_not_watched():
    """그 자리에서 돌기만 하는 goal이다."""
    world = Harness(speed=0.0)
    world.step()
    m = world.traffic._robots["pinky2"]
    m.goal = Goal(MOVE, 99, end="P-2", via=NAV_GOAL)
    world.run(LIMIT + 0.5)
    assert count(world, "NO_START") == 0


def test_the_watch_is_off_at_zero():
    world = called(params=replace(load_params(), start_limit_s=0.0))
    world.run(3 * LIMIT)
    assert count(world, "NO_START") == 0
