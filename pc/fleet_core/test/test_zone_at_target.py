"""goal은 실패했거나 열린 채 멈췄는데 로봇은 그 goal의 끝 지점에 서 있는 경우.

Gazebo 3대 경기에서 나왔다: 출고 도크에서 물러난 로봇이 끝에 닿았지만 후진 goal이 끝나지 않았고,
다시 보낸 후진은 "남은 길이 없다"로 바로 실패해서 로봇이 격리됐다.
"""
import math
from dataclasses import replace

import pytest
from fleet_core.common.commands import CancelCmd, MoveOrderCmd
from fleet_core.common.params import load_params
from zone_harness import Harness

ON = replace(load_params(), use_order_model=True)


def retreat_answer(world):
    """물러나기 goal의 결과를 넣는 함수(robot, goal, success, reason). order 모델이면 물러나기는
    order의 조각이므로 order의 결과다."""
    if world.params.use_order_model:
        return world.traffic.on_move_result
    return world.traffic.on_retreat_result


def silence_retreat(world):
    """물러나기의 결과가 관제에 닿지 않게 하고, 원래 함수를 돌려준다."""
    name = "on_move_result" if world.params.use_order_model else "on_retreat_result"
    answer = getattr(world.traffic, name)
    setattr(world.traffic, name, lambda *args: None)
    return answer


def backing_out(silent_end=True, params=None):
    """pode1이 작업 도크에서 N_W로 물러나는 세계. silent_end면 끝에 닿아도 결과가 오지 않는다.
    order 모델에서 물러나기는 order의 첫 조각이다. silent_end면 로봇은 그 조각 끝에서 말없이 선다.
    answer(robot, goal, success, reason)는 물러나기 goal의 결과를 넣는다."""
    world = Harness(starts={"pode1": "W", "pode2": "P-2", "pode3": "P-3"}, params=params)
    if silent_end and world.params.use_order_model:
        world.hold_after["pode1"] = "retreat"
    world.report()
    world.call("OUTBOUND", "pode1")
    world.run(0.5)
    answer = retreat_answer(world)
    if silent_end and not world.params.use_order_model:
        silence_retreat(world)
    return world, world.retreats("pode1")[0], answer


def stopped(world, robot):
    """로봇이 goal 끝에 섰다: 받은 이동을 끝냈거나, order의 조각 끝에서 말없이 서 있다."""
    bot = world.bots[robot]
    return bot.cmd is None and (bot.run is None or bot.run.held)


def moved_on(world, robot, after):
    """after(goal id) 뒤에 새 이동 goal이 나갔다."""
    return [leg for leg in world.moves(robot) if leg.goal_id > after]


def test_a_retreat_left_open_with_the_robot_at_its_end_is_taken_as_arrived():
    world, retreat, _ = backing_out()
    assert world.run_until(lambda: "GOAL_STALLED" in world.codes("pode1"), 30.0)
    codes = world.codes("pode1")
    assert "GOAL_ENDED_AT_TARGET" in codes and "RETREAT_DONE" in codes
    assert "GOAL_RETRY" not in codes, "standing at the end is not a failed attempt"
    assert [c.goal_id for c in world.sent(CancelCmd, "pode1")] == [retreat.goal_id], \
        "the open goal is cancelled"
    assert world.view("pode1")["node"] == "N_W"
    assert world.run_until(lambda: moved_on(world, "pode1", retreat.goal_id), 10.0), \
        "it goes on from N_W"
    assert len(world.retreats("pode1")) == 1, "the retreat is not sent again"
    assert not world.problems


def test_a_retreat_answered_as_failed_with_the_robot_at_its_end_is_taken_as_arrived():
    world, retreat, answer = backing_out()
    assert world.run_until(lambda: stopped(world, "pode1"), 20.0), "it reached N_W"
    world.step()
    answer("pode1", retreat.goal_id, False, "FAILED")
    world.step()
    codes = world.codes("pode1")
    assert "GOAL_ENDED_AT_TARGET" in codes and "RETREAT_DONE" in codes
    assert "GOAL_RETRY" not in codes
    assert world.view("pode1")["node"] == "N_W"
    assert not world.sent(CancelCmd, "pode1"), "the goal has ended, nothing to cancel"


def test_a_retreat_that_fails_short_of_its_end_is_still_a_failed_attempt():
    world, retreat, answer = backing_out(silent_end=False)
    answer("pode1", retreat.goal_id, False, "FAILED")  # 아직 도크 앞이다
    world.step()
    codes = world.codes("pode1")
    assert "GOAL_RETRY" in codes and "GOAL_ENDED_AT_TARGET" not in codes
    assert world.view("pode1")["node"] == "W"


def stopped_behind_short_of_the_end(order):
    """pode1이 작업 도크에서 N_W로 물러나다가 끝에서 0.14 m 안(도착 허용 거리 0.15 m 안, 갈림목
    안)에 서고, 뒤에 무엇이 있다고 답한다."""
    params = replace(load_params(), use_order_model=order)
    world, retreat, answer = backing_out(silent_end=False, params=params)
    bot, end = world.bots["pode1"], world.site.node_pose("N_W")
    assert world.run_until(lambda: math.dist((bot.x, bot.y), end) <= 0.14, 20.0)
    assert math.dist((bot.x, bot.y), end) > world.params.arrival_tolerance_m / 2
    bot.cmd, bot.run, bot.points = None, None, []
    world.step()
    answer("pode1", retreat.goal_id, False, "OBSTACLE_BEHIND")
    world.step()
    return world, retreat


@pytest.mark.parametrize("order", [False, True], ids=["order_off", "order_on"])
def test_a_retreat_stopped_by_something_behind_short_of_its_end_is_not_taken_as_arrived(order):
    """2026-11-15 order_burst 통합 판: 출고 도크에서 N_E로 물러나던 로봇이 뒤로 지나가는 로봇
    때문에 끝에서 0.2 m 앞(갈림목 안)에 섰다. 잡음 섞인 보고 하나가 끝에서 0.143 m로 보여 닿았다고
    쳤고, 갈림목 안에서 받은 Nav2 goal이 세 번 실패해 격리됐다. 뒤가 막혀 선 로봇은 선을 따라
    남은 길이 허용 거리의 절반을 넘으면 닿은 것이 아니다: 뒤가 빌 때까지 기다린다."""
    world, retreat = stopped_behind_short_of_the_end(order)
    codes = world.codes("pode1")
    assert "GOAL_ENDED_AT_TARGET" not in codes and "RETREAT_DONE" not in codes
    assert "RETREAT_WAIT" in codes and "GOAL_RETRY" not in codes
    assert world.view("pode1")["node"] == "W"
    assert not moved_on(world, "pode1", retreat.goal_id), "no Nav2 goal from inside the junction"


@pytest.mark.parametrize("order", [False, True], ids=["order_off", "order_on"])
def test_a_retreat_stopped_by_something_behind_backs_out_again_when_it_is_gone(order):
    world, retreat = stopped_behind_short_of_the_end(order)
    assert world.run_until(lambda: len(world.retreats("pode1")) == 2, 10.0)
    assert world.run_until(lambda: "RETREAT_DONE" in world.codes("pode1"), 20.0)
    assert world.view("pode1")["node"] == "N_W" and not world.problems


def test_a_robot_sent_somewhere_else_forgets_how_long_something_was_behind():
    """뒤가 막혀 기다리던 로봇의 배차가 취소되면 가던 길은 버리는데 기다린
    시각은 남았다. 나중의 다른 후진이 그 시각부터 blocked_timeout_s를 셌다."""
    world = Harness(starts={"pode1": "W", "pode2": "P-2", "pode3": "P-3"})
    world.report()
    call = world.call("OUTBOUND", "pode1")
    world.run(0.5)
    [retreat] = world.retreats("pode1")
    retreat_answer(world)("pode1", retreat.goal_id, False, "OBSTACLE_BEHIND")
    m = world.traffic._robots["pode1"]
    assert m.avoid.behind_since is not None
    world.traffic.on_dispatch_cancel(call)
    world.traffic.tick_pick()
    assert m.avoid.behind_since is None


def test_a_move_that_arrives_forgets_how_long_something_was_behind():
    """기다린 시각은 이어지는 후진 실패에만 쓴다. 다른 구간이 끝나면 남지 않는다."""
    world = Harness()
    world.step()
    world.call("WORK", "pode2")
    world.run(0.5)
    m = world.traffic._robots["pode2"]
    m.avoid.behind_since = 0.0
    assert world.run_until(lambda: world.view("pode2")["node"] != "P-2", 30.0)
    assert m.avoid.behind_since is None


def test_a_move_left_open_short_of_its_end_is_still_a_stall():
    world = Harness()
    world.step()
    world.call("WORK", "pode2")
    world.run(1.0)
    world.bots["pode2"].cmd = world.bots["pode2"].run = None  # 가다가 섰고 결과도 오지 않는다
    assert world.run_until(lambda: "GOAL_STALLED" in world.codes("pode2"), 30.0)
    codes = world.codes("pode2")
    assert "GOAL_RETRY" in codes and "GOAL_ENDED_AT_TARGET" not in codes


def test_a_move_answered_as_failed_with_the_robot_at_its_end_is_taken_as_arrived():
    world = Harness()
    world.step()
    world.call("WORK", "pode2")
    world.step()
    first = world.moves("pode2")[0]
    world.hold_after["pode2"] = first.mode  # order 모델: 첫 조각 끝에서 말없이 선다
    answer, world.traffic.on_move_result = world.traffic.on_move_result, lambda *args: None
    assert world.run_until(lambda: stopped(world, "pode2"), 60.0), "it reached N_W"
    world.step()
    answer("pode2", first.goal_id, False, "NAV_ABORTED")
    codes = world.codes("pode2")
    world.step()
    assert "GOAL_ENDED_AT_TARGET" in world.codes("pode2") and "GOAL_RETRY" not in codes
    assert world.view("pode2")["node"] == first.target_id


def test_the_next_goal_waits_until_the_cancel_of_the_open_goal_has_reached_the_robot():
    world, retreat, _ = backing_out()
    assert world.run_until(lambda: "GOAL_ENDED_AT_TARGET" in world.codes("pode1"), 30.0)
    world.run(world.params.arrival_settle_s - 0.2)
    assert not moved_on(world, "pode1", retreat.goal_id), "the cancel may still be on its way"
    assert world.run_until(lambda: moved_on(world, "pode1", retreat.goal_id), 2.0)


# ---- 위치를 믿을 수 없으면 닿았다고 보지 않는다 ----

def at_the_end_unanswered():
    world, retreat, answer = backing_out()
    assert world.run_until(lambda: stopped(world, "pode1"), 20.0), "it reached N_W"
    world.step()
    return world, retreat, answer


def test_a_failure_heard_after_the_reports_stopped_is_not_taken_as_arrived():
    world, retreat, answer = at_the_end_unanswered()
    world.silent.add("pode1")
    world.run(world.params.arrival_settle_s + 0.5)  # 아직 COMM_LOST는 아니다
    answer("pode1", retreat.goal_id, False, "FAILED")
    world.step()
    codes = world.codes("pode1")
    assert "GOAL_RETRY" in codes and "GOAL_ENDED_AT_TARGET" not in codes


def test_a_failure_from_a_robot_that_has_lost_its_position_is_not_taken_as_arrived():
    world, retreat, answer = at_the_end_unanswered()
    bot = world.bots["pode1"]
    world.traffic.on_robot_state("pode1", bot.x, bot.y, bot.yaw, "LOCALIZING")
    answer("pode1", retreat.goal_id, False, "FAILED")
    codes = [e.code for e in world.traffic.take_output() if hasattr(e, "code")]
    assert "GOAL_RETRY" in codes and "GOAL_ENDED_AT_TARGET" not in codes


def test_a_robot_that_stopped_itself_without_the_heartbeat_keeps_its_own_handling():
    world, retreat, answer = at_the_end_unanswered()
    answer("pode1", retreat.goal_id, False, "HEARTBEAT_LOST")
    world.step()
    codes = world.codes("pode1")
    assert "GOAL_ENDED_AT_TARGET" not in codes and "GOAL_RETRY" not in codes


# ---- 로봇이 닿았다고 한 것이 아니면 끝에 더 가까워야 한다 ----

def aborted_short_of_the_end(gap_m: float):
    """pode2의 첫 Nav2 goal이 끝 지점에서 gap_m 못 미친 곳에서 실패로 끝난 세계."""
    world = Harness()
    world.step()
    world.call("WORK", "pode2")
    world.step()
    first = world.moves("pode2")[0]
    bot = world.bots["pode2"]
    reach = math.dist((bot.x, bot.y), (first.x, first.y))
    bot.cmd = bot.run = None  # 가다가 섰다. 결과는 아래에서 넣는다
    bot.x = first.x + (bot.x - first.x) * gap_m / reach
    bot.y = first.y + (bot.y - first.y) * gap_m / reach
    world.step()
    world.traffic.on_move_result("pode2", first.goal_id, False, "NAV_ABORTED")
    world.step()
    return world, first


def test_a_failed_move_that_stopped_inside_the_tolerance_but_short_of_the_end_is_retried():
    """관제의 도착 허용 거리와 로봇이 받은 경로에 붙었다고 보는 거리는 같다(0.15 m). 로봇이 닿았다고
    한 것이 아닌데 그 언저리의 보고 하나로 닿은 것으로 보면, 이어 보낸 고정 경로를 로봇이 거절한다
    (ROS 시나리오 shared_lane_failure에서 나왔다: 보고 0.128 m, 실제 약 0.19 m)."""
    world, first = aborted_short_of_the_end(0.10)
    assert world.params.arrival_tolerance_m == 0.15
    codes = world.codes("pode2")
    assert "GOAL_RETRY" in codes and "GOAL_ENDED_AT_TARGET" not in codes
    assert world.view("pode2")["node"] != first.target_id
    assert world.run_until(lambda: world.view("pode2")["node"] == first.target_id, 30.0)
    again = [c for c in world.moves("pode2") if c.target_id == first.target_id]
    assert len(again) == 2 and not again[1].path, "the same Nav2 goal is sent once more"
    assert "GOAL_ENDED_AT_TARGET" not in world.codes("pode2") and not world.problems


def test_a_failed_move_that_stopped_at_the_end_within_its_stop_error_is_taken_as_arrived():
    world, first = aborted_short_of_the_end(0.05)
    codes = world.codes("pode2")
    assert "GOAL_ENDED_AT_TARGET" in codes and "GOAL_RETRY" not in codes
    assert world.view("pode2")["node"] == first.target_id


def test_a_failed_retreat_with_the_robot_beside_its_end_is_still_taken_as_arrived():
    """후진은 선을 따라가므로 끝까지 온 로봇에게 다시 보내면 남은 길이 없어 바로 실패한다
    (pode_fsm의 _too_short). 끝 점에서 옆으로 벗어나 있어도 도착 허용 거리 안이면 닿은 것이다."""
    world, retreat, answer = at_the_end_unanswered()
    world.bots["pode1"].y -= 0.10  # 선의 옆으로. 선을 따라서는 끝에 있다
    world.step()
    answer("pode1", retreat.goal_id, False, "FAILED")
    world.step()
    codes = world.codes("pode1")
    assert "GOAL_ENDED_AT_TARGET" in codes and "RETREAT_DONE" in codes
    assert "GOAL_RETRY" not in codes


# ---- 짧은 고정 경로: 출발점도 끝 지점의 허용 거리 안이다 ----

def backing_into_the_bay():
    """v7: pode1이 칸 입구 E-1에서 P-1로 후진해 들어가려 한다. E-1과 P-1은 0.14 m 떨어져 있어
    도착 허용 거리(0.15 m)보다 가깝다. 후진은 움직이지 않고, 결과도 저절로 오지 않는다."""
    world = Harness(layout="v7", starts={"pode1": "E-1", "pode2": "P-2", "pode3": "P-3"})
    world.report()
    answer = silence_retreat(world)
    world.step()
    retreat = world.retreats("pode1")[0]
    bot = world.bots["pode1"]
    bot.cmd, bot.run, bot.points = None, None, []  # 로봇이 후진을 시작하지 못했다
    bot.x, bot.y = world.site.node_pose("E-1")
    world.step()
    return world, retreat, answer


def test_a_bay_retreat_that_failed_at_the_entrance_is_not_taken_as_parked():
    world, retreat, answer = backing_into_the_bay()
    gap = math.dist(world.site.node_pose("E-1"), world.site.node_pose("P-1"))
    assert gap < world.params.arrival_tolerance_m
    answer("pode1", retreat.goal_id, False, "FAILED")
    world.step()
    codes = world.codes("pode1")
    assert "GOAL_ENDED_AT_TARGET" not in codes and "RETREAT_DONE" not in codes
    assert "GOAL_RETRY" in codes
    assert world.view("pode1")["node"] == "E-1"


def test_a_bay_retreat_left_open_at_the_entrance_is_a_stall_not_an_arrival():
    world, _, _ = backing_into_the_bay()
    assert world.run_until(lambda: "GOAL_STALLED" in world.codes("pode1"), 30.0)
    codes = world.codes("pode1")
    assert "GOAL_ENDED_AT_TARGET" not in codes and "GOAL_RETRY" in codes
    assert world.view("pode1")["node"] == "E-1"


def test_a_bay_retreat_that_failed_most_of_the_way_in_is_taken_as_parked():
    world, retreat, answer = backing_into_the_bay()
    world.bots["pode1"].y = world.site.node_pose("P-1")[1] + 0.03  # 칸 안, 끝에서 0.03 m
    world.step()
    answer("pode1", retreat.goal_id, False, "FAILED")
    world.step()
    codes = world.codes("pode1")
    assert "GOAL_ENDED_AT_TARGET" in codes and "RETREAT_DONE" in codes
    assert world.view("pode1")["node"] == "P-1"


def comm_back_during_the_bay_retreat(short_of_p1_m=None):
    """칸 후진 중에 보고가 끊겼다가 돌아와 관제가 선 자리를 확인한 세계. short_of_p1_m이 있으면
    끊긴 동안 로봇이 칸 안, P-1에서 그만큼 앞으로 옮겨 있다."""
    world, _, answer = backing_into_the_bay()
    world.silent.add("pode1")
    assert world.run_until(lambda: "COMM_LOST" in world.codes("pode1"), 10.0)
    name = "on_move_result" if world.params.use_order_model else "on_retreat_result"
    setattr(world.traffic, name, answer)  # 다시 보낸 후진의 결과는 관제에 닿는다
    if short_of_p1_m is not None:
        world.bots["pode1"].y = world.site.node_pose("P-1")[1] + short_of_p1_m
    world.silent.discard("pode1")
    assert world.run_until(lambda: "COMM_RESTORED" in world.codes("pode1"), 5.0)
    world.run(world.params.recover_check_s + 0.2)
    return world


def test_a_robot_heard_again_at_the_bay_entrance_is_not_taken_as_parked():
    world = comm_back_during_the_bay_retreat()
    assert world.view("pode1")["node"] == "E-1"
    assert len(world.retreats("pode1")) == 2, "the bay retreat is sent again"
    assert world.run_until(lambda: world.phase("pode1") == "PARKED", 10.0)
    assert world.bots["pode1"].y == world.site.node_pose("P-1")[1], "it really backed in"
    assert not world.problems


def test_a_robot_heard_again_most_of_the_way_into_the_bay_is_taken_as_parked():
    world = comm_back_during_the_bay_retreat(short_of_p1_m=0.03)
    assert world.view("pode1")["node"] == "P-1"
    assert world.phase("pode1") == "PARKED"
    assert len(world.retreats("pode1")) == 1


def test_a_short_reverse_link_failed_near_its_end_is_taken_as_reached():
    """후진 링크가 짧으면(0.03) 절반을 못 갔어도 끝에 허용치 절반(0.075) 안이면 닿은 것으로 본다.
    다시 보내면 pode_fsm의 가장 짧은 후진(backup_min_m 0.02)보다 짧아 곧바로 실패한다.
    칸 입구에 그대로 선 로봇(0.14)은 여전히 닿지 않았다."""
    from types import SimpleNamespace

    from fleet_core.fleet_map import RETREAT, Link
    world = Harness(layout="v7", starts={"pode1": "P-1"})
    traffic, limit = world.traffic, world.params.arrival_tolerance_m
    short = Link("", "P-1", RETREAT, "", ((0.12, 0.13), (0.12, 0.10)), ("HALL",), 0.03)
    assert traffic._link_reached(SimpleNamespace(x=0.12, y=0.118), short, limit)
    bay = world.site.link("E-1", "P-1")
    assert not traffic._link_reached(SimpleNamespace(x=0.12, y=0.24), bay, limit)
    assert traffic._link_reached(SimpleNamespace(x=0.12, y=0.13), bay, limit)


# ---- order 안의 끝내지 않은 조각 끝에 서 있는 로봇 ----

def parking_into_p1(stand_at):
    """v7 order: pode1이 W에서 칸으로 간다. 마지막 update의 base는 [3 nav_goal E-1, 4 retreat
    P-1]이고 E-1과 P-1은 0.14 m(도착 허용 거리 0.15 m 안)다. 로봇은 stand_at까지 가서 끝냈다는
    말 없이 선다(E-1이면 seq 3 끝, P-1이면 seq 4 끝). 끝낸 seq는 2까지만 관제에 닿았다."""
    world = Harness(layout="v7", starts={"pode1": "W", "pode2": "P-2", "pode3": "P-3"},
                    params=ON)
    world.report()
    world.traffic.on_release("pode1")
    bot = world.bots["pode1"]
    assert world.run_until(lambda: bot.run is not None and bot.run.current is not None
                           and bot.run.current.seq_id == 3, 120.0), world.log[-10:]
    [update] = [o for o in world.sent(MoveOrderCmd, "pode1") if o.update_id == 1]
    assert [(s.seq_id, s.mode, s.end_node) for s in update.base] == \
        [(3, "nav_goal", "E-1"), (4, "retreat", "P-1")]
    if stand_at == "E-1":
        world.hold_after["pode1"] = "nav_goal"
        assert world.run_until(lambda: bot.run.held, 30.0)
    else:
        world.hold_answers = True  # 로봇의 피드백과 결과가 관제에 닿지 않는다
        assert world.run_until(lambda: bot.run is None, 30.0)
    assert math.dist((bot.x, bot.y), world.site.node_pose(stand_at)) < 1e-6
    world.step()
    assert world.traffic._robots["pode1"].move_order.done_seq == 2
    return world, update


def test_an_order_that_fails_with_the_robot_at_an_unfinished_segment_end_is_taken_as_arrived():
    """끈 길은 goal이 실패해도 끝에 서 있으면 닿은 것으로 본다(GOAL_ENDED_AT_TARGET). order는 base
    끝(P-1)만 보았으므로 E-1에 선 로봇을 실패한 시도로 셌다. 가까운 두 끝 가운데 로봇이 실제로 닿은
    E-1이다: P-1로 가는 후진 링크의 방향 검사가 E-1에 선 로봇을 P-1에 닿았다고 보지 않는다."""
    world, update = parking_into_p1("E-1")
    world.traffic.on_move_result("pode1", update.goal_id, False, "NAV_ABORTED", 2)
    world.step()
    codes = world.codes("pode1")
    assert "GOAL_ENDED_AT_TARGET" in codes and "GOAL_RETRY" not in codes
    assert world.view("pode1")["node"] == "E-1"
    assert "RETREAT_DONE" not in codes and world.phase("pode1") != "PARKED"


def test_an_order_that_fails_with_the_robot_at_the_base_end_behind_a_close_segment_end_parks():
    """seq 3(E-1)과 seq 4(물러나 P-1)를 끝냈다는 말이 둘 다 오지 않았다. 건너뛴 두 조각도 피드백으로
    끝낸 것처럼 SEGMENT_DONE과 RETREAT_DONE을 내고 물러날 곳을 지운다."""
    world, update = parking_into_p1("P-1")
    before = world.codes("pode1").count("SEGMENT_DONE")
    world.traffic.on_move_result("pode1", update.goal_id, False, "NAV_ABORTED", 2)
    world.step()
    codes = world.codes("pode1")
    assert "GOAL_ENDED_AT_TARGET" in codes and codes.count("RETREAT_DONE") == 1
    assert codes.count("SEGMENT_DONE") == before + 2
    assert world.traffic._robots["pode1"].avoid.retreat_to == ""
    assert "GOAL_RETRY" not in codes
    assert world.view("pode1")["node"] == "P-1" and world.phase("pode1") == "PARKED"


def test_an_order_left_open_at_an_unfinished_segment_end_is_taken_as_arrived_when_it_stalls():
    world, update = parking_into_p1("E-1")
    assert world.run_until(lambda: "GOAL_STALLED" in world.codes("pode1"), 30.0)
    codes = world.codes("pode1")
    assert "GOAL_ENDED_AT_TARGET" in codes and "GOAL_RETRY" not in codes
    assert world.view("pode1")["node"] == "E-1"
    assert [c.goal_id for c in world.sent(CancelCmd, "pode1")] == [update.goal_id]


def test_late_feedback_of_an_order_already_taken_as_arrived_changes_nothing():
    """닿은 것으로 본 뒤에 늦게 온 그 goal의 피드백(seq 3 끝냄)은 버린다. 다시 SEGMENT_DONE을
    내거나 선 자리를 바꾸지 않고, 로봇은 새 goal로 칸에 들어간다."""
    world, update = parking_into_p1("E-1")
    world.traffic.on_move_result("pode1", update.goal_id, False, "NAV_ABORTED", 2)
    world.step()
    done = world.codes("pode1").count("SEGMENT_DONE")
    world.traffic.on_move_feedback("pode1", update.goal_id, 3, False)
    world.traffic.on_move_feedback("pode1", update.goal_id, 4, False)
    world.traffic.on_move_result("pode1", update.goal_id, False, "NAV_ABORTED", 3)
    world.step()
    assert world.codes("pode1").count("SEGMENT_DONE") == done
    assert "GOAL_RETRY" not in world.codes("pode1")
    assert world.view("pode1")["node"] == "E-1"
    world.bots["pode1"].run = None  # 로봇은 실패한 order를 버렸다
    world.hold_after.clear()
    assert world.run_until(lambda: world.phase("pode1") == "PARKED", 60.0), world.log[-10:]
    assert world.bots["pode1"].y == world.site.node_pose("P-1")[1] and not world.problems


def test_with_two_segment_ends_inside_the_tolerance_the_one_nearer_the_robot_is_taken():
    """E-1(nav_goal 끝)과 P-1(물러나기 끝)이 둘 다 닿았다고 볼 수 있는 자리(E-1에서 0.068 m, P-1에서
    0.072 m)면 더 가까운 E-1이다. 먼 끝부터 고르면 물러나지 않은 로봇을 칸 안에 있다고 본다."""
    world, update = parking_into_p1("E-1")
    bot = world.bots["pode1"]
    (ex, ey), (px, py) = world.site.node_pose("E-1"), world.site.node_pose("P-1")
    t = 0.068 / math.dist((ex, ey), (px, py))
    bot.x, bot.y = ex + (px - ex) * t, ey + (py - ey) * t
    world.step()
    world.traffic.on_move_result("pode1", update.goal_id, False, "NAV_ABORTED", 2)
    world.step()
    assert "GOAL_ENDED_AT_TARGET" in world.codes("pode1")
    assert world.view("pode1")["node"] == "E-1" and world.phase("pode1") != "PARKED"
