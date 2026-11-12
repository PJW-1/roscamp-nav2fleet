"""회복 동작의 순서(추가 지시서 2의 4절): 그 자리에 멈춤 → 지나온 길로 물러나기 → 가까운 주차
자리로. 물러나기는 뒤에 다른 로봇이 없을 때만 한다."""
from dataclasses import replace

from fleet_core.common.commands import MoveCmd, RetreatCmd
from fleet_core.common.params import load_params
from zone_harness import Harness

TWO = {"pinky1": "P-1", "pinky2": "P-2"}  # 세 번째 로봇이 없다: P-3이 비어 있다

# 규칙 6(가까이서 못 나아감)만 본다: 멈춤 감시와 막힘 표시는 끼어들지 않게 멀리 둔다
ONLY_RULE_6 = replace(load_params(), stall_s=1000.0, blocked_mark_s=1000.0)


# ---- 고정 구간 안: 뒤에 로봇이 있으면 물러나지 않는다 ----

def blocked_in_the_express():
    """W에서 출고 도크로 가던 pinky1이 급행 안의 상자 앞에 서서 BLOCKED를 보고했다."""
    world = Harness(starts={"pinky1": "W", "pinky2": "P-2", "pinky3": "P-3"})
    world.report()
    call = world.call("OUTBOUND", "pinky1")
    world.obstacles.append((1.0, 0.89, 0.15))
    assert world.run_until(lambda: "BLOCKED_WAIT" in world.codes("pinky1"), 60.0)
    return world, call


def test_a_robot_blocked_in_the_express_does_not_back_out_while_another_is_behind_it():
    world, call = blocked_in_the_express()
    behind = world.bots["pinky2"]
    behind.x, behind.y = 0.62, 0.89  # 급행 안, pinky1이 물러날 길 위
    retreats = len(world.sent(RetreatCmd, "pinky1"))
    world.run(world.params.blocked_mark_s + 5.0)
    assert len(world.sent(RetreatCmd, "pinky1")) == retreats, "it would back into pinky2"
    waits = [(e.robot, e.detail) for e in world.events if e.code == "RETREAT_WAIT"]
    assert waits == [("pinky1", "pinky2 is behind it, backs out after it")]
    assert "ZONE_MARKED_BLOCKED" not in world.codes("pinky1")

    behind.x, behind.y = world.site.node_pose("P-2")  # 뒤의 로봇이 먼저 나갔다
    world.run(1.0)
    assert len(world.sent(RetreatCmd, "pinky1")) == retreats + 1
    assert world.run_until(lambda: world.result_of(call), 120.0)
    assert world.result_of(call).success


def test_a_robot_beside_the_way_back_does_not_hold_the_retreat_up():
    world, call = blocked_in_the_express()
    beside = world.bots["pinky2"]
    beside.x, beside.y = world.site.node_pose("N_W")  # 갈림목 아래 마당: 물러날 길에서 0.24 m
    assert world.run_until(lambda: world.result_of(call), 120.0)
    assert world.result_of(call).success
    assert "RETREAT_WAIT" not in world.codes()


def test_a_robot_ahead_of_it_is_not_taken_for_one_behind():
    """가는 쪽에 바짝 붙은 로봇은 물러날 길 위에 있지 않다. 그런 로봇까지 뒤로 보면 급행 안에서
    가까이 선 두 대가 서로를 기다린다."""
    world, _ = blocked_in_the_express()
    blocked, ahead = world.bots["pinky1"], world.bots["pinky2"]
    ahead.x, ahead.y = blocked.x + 0.15, blocked.y
    retreats = len(world.sent(RetreatCmd, "pinky1"))
    world.run(world.params.blocked_mark_s + 2.0)
    assert len(world.sent(RetreatCmd, "pinky1")) == retreats + 1
    assert "RETREAT_WAIT" not in world.codes()


def test_a_robot_behind_that_never_leaves_ends_the_wait_with_a_failed_goal():
    world, call = blocked_in_the_express()
    behind = world.bots["pinky2"]
    behind.x, behind.y = 0.62, 0.89
    retreats = len(world.sent(RetreatCmd, "pinky1"))
    world.run(world.params.blocked_timeout_s + 2.0)
    failed = [e.detail for e in world.events if e.code == "GOAL_FAILED" and e.robot == "pinky1"]
    assert failed == ["blocked for 60 s with pinky2 behind it"]
    assert world.view("pinky1")["fault"]
    assert not world.result_of(call).success
    assert len(world.sent(RetreatCmd, "pinky1")) == retreats, "it never backed into pinky2"


def test_a_new_goal_tells_again_that_it_waits_for_the_robot_behind():
    world, _ = blocked_in_the_express()
    behind = world.bots["pinky2"]
    behind.x, behind.y = 0.62, 0.89
    world.run(world.params.blocked_mark_s + 1.0)
    assert world.codes("pinky1").count("RETREAT_WAIT") == 1
    m = world.traffic._robots["pinky1"]
    m.goal = replace(m.goal, id=m.goal.id + 1000)  # BLOCKED를 보고하는 채로 goal이 바뀌었다
    world.run(world.params.blocked_mark_s + 1.0)
    assert world.codes("pinky1").count("RETREAT_WAIT") == 2


# ---- 넓은 구역 안: 같은 자리에서 또 비키면 주차 자리로 ----

def stuck_pair(starts=None):
    """P-1의 pinky1과 P-2의 pinky2가 0.25 m 떨어져 마당에서 Nav2 goal을 받고 서 있다.
    배차가 없는 pinky2가 비킬 쪽이다."""
    world = Harness(speed=0.0, params=ONLY_RULE_6, starts=starts)
    world.step()
    world.call("WORK", "pinky1")
    world.call("WORK")  # 열린 호출: pinky1은 이미 배차가 있어 pinky2가 불린다
    world.step()
    world.step()
    assert world.sent(MoveCmd, "pinky1") and world.sent(MoveCmd, "pinky2")
    return world


def yields(world):
    return [(e.robot, e.detail) for e in world.events if e.code == "HALL_YIELD"]


def test_giving_way_twice_at_the_same_place_sends_the_robot_to_a_parking_spot():
    world = stuck_pair(TWO)
    p = world.params
    world.run(p.near_stuck_s)
    assert yields(world) == [("pinky2", "stops 5 s for pinky1")]
    world.run(2 * p.near_stuck_s + 0.5)  # 서 있다가 같은 길을 다시 받고, 또 못 나아간다
    assert yields(world)[1:] == [("pinky2", "out of the way of pinky1, goes to P-3")]
    assert world.sent(MoveCmd, "pinky2")[-1].target_id == "P-3"


def test_with_no_parking_spot_to_go_to_it_stops_again():
    world = stuck_pair()  # P-1과 P-3에는 다른 로봇이 있고, P-2는 제가 선 자리다
    p = world.params
    world.run(3 * p.near_stuck_s + 0.5)
    assert yields(world) == [("pinky2", "stops 5 s for pinky1")] * 2


def test_a_robot_that_got_away_and_is_stuck_somewhere_else_starts_with_a_stop_again():
    world = stuck_pair(TWO)
    p = world.params
    world.run(p.near_stuck_s)
    for name in ("pinky1", "pinky2"):  # 둘 다 동쪽으로 0.5 m 간 곳에서 다시 붙었다
        world.bots[name].x += 0.5
    world.run(2 * p.near_stuck_s + 0.5)
    assert yields(world) == [("pinky2", "stops 5 s for pinky1")] * 2
