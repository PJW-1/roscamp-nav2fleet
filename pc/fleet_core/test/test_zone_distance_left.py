"""order 모델에서 Nav2 goal 조각의 남은 시간은 피드백의 남은 거리(distance_remaining)로 잰다. 값이 없거나 믿을 수 없으면 투영(left_s)으로 재고, 끝 보정(nav_end_s)은 똑같이 붙는다."""
import json
import math
import sys
from dataclasses import replace
from pathlib import Path

import pytest
from fleet_core.common.commands import MoveOrderCmd, OrderRecord
from fleet_core.common.params import load_params
from fleet_core.fleet_map import NAV_GOAL, load_fleet_map
from fleet_core.manager import FleetManager
from fleet_core.traffic.zone import order_answers
from fleet_core.traffic.zone.routes import left_s, nav_end_s, nav_stretch, speed, travel_s
from zone_harness import Harness

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
import replay  # noqa: E402

ON = replace(load_params(), use_order_model=True)


def driving_to_n_w():
    """pode2가 작업 도크로 불려 첫 조각(Nav2 goal, N_W까지)을 막 시작했다."""
    world = Harness(params=ON)
    world.step()
    world.call("WORK")
    world.step()
    [order] = world.sent(MoveOrderCmd, "pode2")
    assert [(s.seq_id, s.mode) for s in order.base] == [(1, NAV_GOAL), (2, "follow_path")]
    return world, order.goal_id, world.traffic._robots["pode2"]


def eta(world, m):
    return world.traffic._left_s(m, m.route, m.route.granted)


def chain_end(m):
    route = m.route
    return route.done + nav_stretch(route.edges[route.done:route.granted])


def with_end(world, m, nav_s, chain):
    route = m.route
    adjust = nav_end_s(world.params, world.site, route.edges, chain, m.x, m.y, m.yaw)
    return max(0.0, nav_s + adjust) + travel_s(world.params, route.edges[chain:route.granted])


def projected(world, m):
    route = m.route
    if route.edges[route.done].mode != NAV_GOAL:
        return left_s(world.params, route.edges[route.done:route.granted], m.x, m.y)
    chain = chain_end(m)
    return with_end(world, m, left_s(world.params, route.edges[route.done:chain], m.x, m.y), chain)


def from_distance(world, m, distance):
    route, chain = m.route, chain_end(m)
    _, _, end = m.move_order.cuts[m.move_order.done_seq]
    nav_s = (distance / speed(world.params, NAV_GOAL)
             + travel_s(world.params, route.edges[end:chain]))
    return with_end(world, m, nav_s, chain)


def feed(world, goal_id, distance, done=0, waiting=False, dt=0.2):
    world.now += dt
    world.traffic.on_move_feedback("pode2", goal_id, done, waiting, "", distance)


# ---- 쓰는 경우 ----

def test_decreasing_values_of_the_current_nav_goal_segment_are_used():
    world, goal_id, m = driving_to_n_w()
    feed(world, goal_id, 0.80)
    assert eta(world, m) == pytest.approx(from_distance(world, m, 0.80))
    feed(world, goal_id, 0.78)
    assert eta(world, m) == pytest.approx(from_distance(world, m, 0.78))
    assert eta(world, m) != pytest.approx(projected(world, m)), "a value unlike the projection"


def test_a_detour_longer_than_the_graph_is_believed():
    world, goal_id, m = driving_to_n_w()
    feed(world, goal_id, projected(world, m) * speed(world.params, NAV_GOAL) + 1.0)
    assert eta(world, m) > projected(world, m) + 1.0 / speed(world.params, NAV_GOAL) - 1e-6


# ---- 투영으로 돌아가는 경우 ----

def test_no_value_or_no_feedback_yet_falls_back_to_the_projection():
    world, goal_id, m = driving_to_n_w()
    assert eta(world, m) == pytest.approx(projected(world, m)), "no feedback yet"
    feed(world, goal_id, 0.80)
    feed(world, goal_id, None)  # 옛 로봇과 옛 기록
    assert eta(world, m) == pytest.approx(projected(world, m))


def test_a_stale_value_falls_back_to_the_projection():
    world, goal_id, m = driving_to_n_w()
    feed(world, goal_id, 0.80)
    world.now += order_answers.LEFT_STALE_S - 0.05
    assert eta(world, m) == pytest.approx(from_distance(world, m, 0.80))
    world.now += 0.1
    assert eta(world, m) == pytest.approx(projected(world, m))


def test_a_replan_jump_falls_back_until_the_value_stays_calm():
    world, goal_id, m = driving_to_n_w()
    feed(world, goal_id, 0.80)
    feed(world, goal_id, 0.80 + order_answers.LEFT_JUMP_M - 0.01)  # 잡음 안쪽
    assert eta(world, m) == pytest.approx(from_distance(world, m, 0.89))
    feed(world, goal_id, 0.89 + order_answers.LEFT_JUMP_M + 0.01)  # 길이 뒤집혔다
    assert eta(world, m) == pytest.approx(projected(world, m))
    calm, value = 0.2, 1.0
    while calm < order_answers.LEFT_CALM_S - 1e-9:
        value -= 0.02
        feed(world, goal_id, value)
        assert eta(world, m) == pytest.approx(projected(world, m)), f"{calm:.1f} s after"
        calm += 0.2
    value -= 0.02
    feed(world, goal_id, value)
    assert eta(world, m) == pytest.approx(from_distance(world, m, value))


@pytest.mark.parametrize("gap", [None, math.nan, -0.1], ids=["missing", "nan", "negative"])
def test_a_bad_sample_after_a_jump_does_not_end_the_calm_wait(gap):
    world, goal_id, m = driving_to_n_w()
    feed(world, goal_id, 0.80)
    feed(world, goal_id, 1.00)  # 길이 뒤집혔다
    feed(world, goal_id, gap)
    assert eta(world, m) == pytest.approx(projected(world, m))
    feed(world, goal_id, 0.98)
    assert eta(world, m) == pytest.approx(projected(world, m)), "still inside the calm time"


@pytest.mark.parametrize("bad", [math.nan, math.inf, -0.05], ids=["nan", "inf", "negative"])
def test_a_value_that_is_not_a_distance_is_not_used(bad):
    world, goal_id, m = driving_to_n_w()
    world.bots["pode2"].x, world.bots["pode2"].y = world.site.node_pose("N_W")
    m.x, m.y = world.site.node_pose("N_W")
    feed(world, goal_id, bad)
    assert eta(world, m) == pytest.approx(projected(world, m))
    assert eta(world, m) >= 0.0


def test_flipping_back_and_forth_keeps_falling_back():
    world, goal_id, m = driving_to_n_w()
    for step in range(20):
        feed(world, goal_id, 0.80 if step % 5 else 1.10)
        if step >= 5:
            assert eta(world, m) == pytest.approx(projected(world, m)), f"sample {step}"


def test_a_value_shorter_than_the_straight_line_falls_back():
    world, goal_id, m = driving_to_n_w()
    straight = math.dist((m.x, m.y), world.site.node_pose("N_W"))
    feed(world, goal_id, straight - order_answers.LEFT_SHORT_M + 0.01)
    assert eta(world, m) == pytest.approx(from_distance(world, m, straight - 0.09))
    feed(world, goal_id, straight - order_answers.LEFT_SHORT_M - 0.01)
    assert eta(world, m) == pytest.approx(projected(world, m))


def test_a_repeated_same_value_stays_fresh_and_repeated_feedback_says_things_once():
    world, goal_id, m = driving_to_n_w()
    for _ in range(int(2 * order_answers.LEFT_STALE_S / 0.2) + 1):
        feed(world, goal_id, 0.80)  # 서 있는 로봇: 같은 피드백이 거듭 온다
    assert eta(world, m) == pytest.approx(from_distance(world, m, 0.80))
    for _ in range(5):
        feed(world, goal_id, 0.0, waiting=True)
    world.step()
    assert world.codes("pode2").count("BASE_END_WAIT") == 1


def test_waiting_at_the_base_end_is_not_a_distance():
    world, goal_id, m = driving_to_n_w()
    feed(world, goal_id, 0.80)
    feed(world, goal_id, 0.0, waiting=True)
    assert eta(world, m) == pytest.approx(projected(world, m))


def test_a_fixed_path_segment_keeps_the_projection():
    world, goal_id, m = driving_to_n_w()
    feed(world, goal_id, 0.05, done=1)  # 2번은 고정 경로다
    assert m.move_order.done_seq == 1
    assert eta(world, m) == pytest.approx(projected(world, m))


def test_a_value_of_a_finished_segment_is_not_used_for_the_next_one():
    """d2: 3번(Nav2 goal, N_E까지)을 받은 뒤 2번의 늦은 피드백은 3번의 남은 거리로 쓰지 않는다."""
    world = Harness(starts={"pode1": "W", "pode2": "O", "pode3": "P-3"}, params=ON)
    world.report()
    world.call("OUTBOUND", "pode2")
    world.call("OUTBOUND", "pode1")
    assert world.run_until(lambda: "BASE_END_WAIT" in world.codes("pode1"), 60.0)
    world.traffic.on_release("pode2")
    assert world.run_until(lambda: len(world.sent(MoveOrderCmd, "pode1")) == 2, 60.0)
    update = world.sent(MoveOrderCmd, "pode1")[-1]
    assert [(s.seq_id, s.mode) for s in update.base][0] == (3, NAV_GOAL)
    m = world.traffic._robots["pode1"]
    assert m.move_order.done_seq == 2
    world.now += 0.1
    world.traffic.on_move_feedback("pode1", update.goal_id, 1, False, "", 0.40)  # 2번의 값
    assert eta(world, m) == pytest.approx(projected(world, m))
    world.now += 0.1
    world.traffic.on_move_feedback("pode1", update.goal_id, 2, False, "", 0.40)  # 3번의 값
    assert eta(world, m) == pytest.approx(from_distance(world, m, 0.40))


# ---- 남은 거리가 base를 늘리는 때를 정한다 ----

def time_of_the_extension_to_q1(detour_m):
    """v7: 남은 거리가 곧은 거리보다 detour_m 길 때(None이면 값 없음) 3번(Q1)을 base로 늘리는 ORDER_UPDATED가 나가는 때를 본다."""
    world = Harness(layout="v7", params=ON, starts={"pode1": "P-1", "pode2": "P-2", "pode3": "P-3"})
    world.report()
    world.call("WORK", "pode1")
    m = world.traffic._robots["pode1"]
    assert world.run_until(lambda: m.move_order is not None and m.move_order.done_seq == 1, 60.0)
    q2 = world.site.node_pose("Q2")
    while "ORDER_UPDATED" not in world.codes("pode1"):
        assert world.now < 60.0
        if detour_m is not None:
            world.traffic.on_move_feedback("pode1", m.goal.id, 1, False, "",
                                           math.dist((m.x, m.y), q2) + detour_m)
        world.step()
    return world.now


def test_a_longer_way_left_holds_the_extension_back_by_its_driving_time():
    straight = time_of_the_extension_to_q1(0.0)
    longer = time_of_the_extension_to_q1(0.15)
    assert longer - straight == pytest.approx(0.15 / ON.nav_speed_mps, abs=0.15)
    plain = time_of_the_extension_to_q1(None)
    assert plain < longer - 0.5, "the projection does not see the longer way"


# ---- 기록과 재생 ----

def recorded_run(with_distance):
    now = [0.0]
    core = FleetManager(load_fleet_map("d2"), ON, ["pode1"], lambda: now[0])
    lines = []
    core.record_to(lines.append)
    for _ in range(ON.release_confirm_samples):
        core.on_robot_state("pode1", *core.site.node_pose("P-1"), 0.0, "IDLE")
    core.on_order_board("RUN", [OrderRecord("O-001", {"A": 1})])
    order = None
    while order is None and now[0] < 5.0:
        now[0] += 0.1
        order = next((c for c in core.tick() if isinstance(c, MoveOrderCmd)), None)
    assert order is not None and order.base[0].mode == NAV_GOAL
    for step in range(10):
        now[0] += 0.1
        extra = (0.9 - 0.02 * step,) if with_distance else ()
        core.on_move_feedback("pode1", order.goal_id, 0, False, "", *extra)
        core.tick()
    return lines


def feedback_args(lines):
    return [json.loads(line)["args"] for line in lines if '"on_move_feedback"' in line]


def play(tmp_path, lines):
    path = tmp_path / "run.jsonl"
    path.write_text("".join(line + "\n" for line in lines), encoding="utf-8")
    return replay.replay(path)


def test_a_record_without_the_distance_keeps_the_old_five_arguments_and_plays_back(tmp_path):
    lines = recorded_run(with_distance=False)
    assert {len(args) for args in feedback_args(lines)} == {5}, "the format before 2026-11-21"
    result = play(tmp_path, lines)
    assert result.same and result.compared, result


def test_a_record_with_the_distance_plays_back(tmp_path):
    lines = recorded_run(with_distance=True)
    assert {len(args) for args in feedback_args(lines)} == {6}
    result = play(tmp_path, lines)
    assert result.same and result.compared, result
