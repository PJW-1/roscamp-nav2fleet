"""order 모델의 가짜 로봇(계약 0.11-draft 제안): base를 잇달아 달리고, base 끝에서 서고,
조건 넷이 맞는 update만 붙인다. 관제는 대역이라 시험이 명령을 직접 넣는다."""
import math

import pytest
from fake_robots.world import CommLoss, Obstacle, Stuck
from fleet_core.common.commands import MoveOrderCmd, SegmentOut
from world_helpers import quiet_world


def seg(world, seq, nodes, mode="follow_path"):
    path = tuple((*world.site.node_pose(n), 0.0) for n in nodes)
    x, y = world.site.node_pose(nodes[-1])
    return SegmentOut(seq, mode, nodes[-1], (), path if mode != "nav_goal" else (), x, y, 0.0)


def order(goal_id, base, horizon=(), update=0, stitch="", order_id="o-1"):
    return MoveOrderCmd("pinky1", goal_id, order_id, update, stitch, tuple(base), tuple(horizon))


def answers(world, method):
    return [args[1:] for _, args in world.manager.of(method, "pinky1")]


def at(world, node):
    return math.dist(world.robots["pinky1"].pose, world.site.node_pose(node)) < 0.02


def test_base_segments_run_back_to_back_and_the_order_ends_with_the_last_seq():
    world = quiet_world()
    world.send(order(1, [seg(world, 1, ["P-1", "N_W"]), seg(world, 2, ["N_W", "X_W"])]))
    assert world.run_until(lambda: answers(world, "on_move_result"), timeout_s=60)
    assert answers(world, "on_move_result") == [(1, True, "", 2)]
    assert at(world, "X_W")
    assert (1, 1, False) in [a[:3] for a in answers(world, "on_move_feedback")]


def _standing_at_base_end(world):
    world.send(order(1, [seg(world, 1, ["P-1", "N_W"])], [seg(world, 2, ["N_W", "X_W"])]))
    world.run(20.0)
    assert at(world, "N_W")
    assert answers(world, "on_move_feedback")[-1][:3] == (1, 1, True)
    assert not answers(world, "on_move_result")


def test_with_a_horizon_left_the_robot_stands_at_the_base_end_and_says_so():
    _standing_at_base_end(quiet_world())


def test_a_good_update_is_stitched_and_the_old_goal_ends_superseded():
    world = quiet_world()
    _standing_at_base_end(world)
    world.send(order(2, [seg(world, 2, ["N_W", "X_W"])], update=1, stitch="N_W"))
    assert world.run_until(lambda: len(answers(world, "on_move_result")) >= 2, timeout_s=30)
    assert answers(world, "on_move_result") == [(1, False, "SUPERSEDED", 1), (2, True, "", 2)]
    assert at(world, "X_W")


@pytest.mark.parametrize("change, why", [
    ({"order_id": "o-2"}, "order_id"),
    ({"update": 2}, "update_id"),
    ({"stitch": "X_W"}, "stitch_node"),
    ({"seq": 3}, "seq_id"),
])
def test_an_update_that_breaks_a_condition_is_refused_and_the_robot_keeps_its_order(change, why):
    world = quiet_world()
    _standing_at_base_end(world)
    kw = {"update": 1, "stitch": "N_W", "order_id": "o-1", **change}
    seq = kw.pop("seq", 2)
    world.send(order(2, [seg(world, seq, ["N_W", "X_W"])], **kw))
    world.run(1.0)
    (goal, ok, reason, done), = answers(world, "on_move_result")
    assert (goal, ok, done) == (2, False, 1)
    assert reason.startswith("ORDER_UPDATE_REJECTED") and why in reason
    assert world.robots["pinky1"].run.goal_id == 1 and at(world, "N_W")


def test_an_update_with_no_order_running_is_refused():
    world = quiet_world()
    world.send(order(2, [seg(world, 2, ["N_W", "X_W"])], update=1, stitch="N_W"))
    world.run(0.5)
    assert answers(world, "on_move_result")[0][:2] == (2, False)


def test_without_contact_the_robot_drives_on_to_the_base_end_under_base_end():
    world = quiet_world(injections=[CommLoss("pinky1", 0.5, 60.0)],
                        overrides={"comm_loss_policy": "base_end"})
    world.send(order(1, [seg(world, 1, ["P-1", "N_W"]), seg(world, 2, ["N_W", "X_W"])],
                     [seg(world, 3, ["X_W", "W"])]))
    world.run(30.0)
    assert at(world, "X_W")
    assert all(t < 0.5 for t, _ in world.manager.of("on_move_feedback", "pinky1"))


def test_without_contact_the_robot_stops_at_once_under_stop():
    """The result is held until contact is back; last_done_seq_id is the last finished segment."""
    world = quiet_world(injections=[CommLoss("pinky1", 3.0, 20.0)])
    assert world.params.comm_loss_policy == "stop"
    world.send(order(1, [seg(world, 1, ["P-1", "N_W"]), seg(world, 2, ["N_W", "X_W"])],
                     [seg(world, 3, ["X_W", "W"])]))
    world.run(2.9)
    r = world.robots["pinky1"]
    run = r.run
    assert run is not None and not at(world, "X_W")
    world.run(1.6)  # silent for heartbeat_timeout_s
    assert r.run is None, "stopped"
    stopped = r.pose
    world.run(10.0)
    assert r.pose == stopped and not at(world, "X_W")
    assert not answers(world, "on_move_result"), "the result waits for contact"
    world.run(8.0)
    assert answers(world, "on_move_result") == [(1, False, "HEARTBEAT_LOST", run.done_seq)]


def test_a_stuck_robot_without_contact_still_ends_with_heartbeat_lost():
    world = quiet_world(injections=[CommLoss("pinky1", 1.0, 20.0), Stuck("pinky1", 1.5)])
    world.send(order(1, [seg(world, 1, ["P-1", "N_W"]), seg(world, 2, ["N_W", "X_W"])],
                     [seg(world, 3, ["X_W", "W"])]))
    world.run(25.0)
    [(goal, ok, reason, _)] = answers(world, "on_move_result")
    assert (goal, ok, reason) == (1, False, "HEARTBEAT_LOST")


def test_an_obstacle_on_the_base_stops_the_robot_short_of_it():
    world = quiet_world()
    (ax, ay), (bx, by) = world.site.node_pose("P-1"), world.site.node_pose("N_W")
    box = ((ax + bx) / 2, (ay + by) / 2)
    world = quiet_world(injections=[Obstacle(*box, 0.04)])
    world.send(order(1, [seg(world, 1, ["P-1", "N_W"])]))
    world.run(10.0)
    assert math.dist(world.robots["pinky1"].pose, box) >= world.radius + 0.04
    assert not answers(world, "on_move_result")
