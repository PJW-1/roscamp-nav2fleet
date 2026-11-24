"""IR-62: goal 결과를 잃으면 결과 모름. 구역을 쥔 채 상태 보고로 한 번만 정하고, 못 정하면 격리한다."""
from fleet_core.common.commands import DockCmd, MoveCmd, MoveOrderCmd, RobotIsolated
from zone_harness import Harness


def sent_once(speed=0.0):
    """pode2가 작업 도크로 불려 첫 Nav2 goal(N_W)을 받았다. speed 0이면 로봇은 그 자리에 선다."""
    world = Harness(speed=speed)
    world.step()
    call = world.call("WORK", "pode2")
    world.step()
    assert [leg.target_id for leg in world.moves("pode2")] == ["N_W"]
    return world, call


def last_goal(world, robot="pode2"):
    return world.sent((MoveCmd, MoveOrderCmd), robot)[-1].goal_id


def moves(world, robot="pode2"):
    return len(world.moves(robot))


def isolations(world, reason=""):
    return [c for c in world.commands
            if isinstance(c, RobotIsolated) and (not reason or c.reason == reason)]


def lose(world, goal_id, robot="pode2"):
    world.traffic.on_goal_lost(robot, goal_id)
    return goal_id


def stop_quietly(world, robot="pode2", at=None):
    """로봇이 goal을 끝내고 서지만 결과는 관제에 오지 않는다. at이면 그 노드에 선다."""
    bot = world.bots[robot]
    if at is not None:
        bot.x, bot.y = world.site.node_pose(at)
    bot.cmd, bot.run, bot.points = None, None, []


def reports(world, robot, state):
    plain = Harness._state.__get__(world)
    world._state = lambda bot: state if bot.name == robot else plain(bot)


def test_a_lost_result_keeps_the_goal_and_its_zones_and_times_out_into_an_isolation():
    world, call = sent_once()
    p = world.params
    held = world.traffic.grants.held_by("pode2")
    lost = lose(world, last_goal(world))
    assert world.view("pode2")["goal_id"] == lost, "not forgotten at once"
    world.step()
    assert world.codes("pode2")[-1] == "ROBOT_TIMEOUT"
    world.run(p.robot_timeout_settle_s - 0.5)
    codes = world.codes("pode2")
    assert moves(world) == 1, "no new goal while the result is unknown"
    assert world.view("pode2")["goal_id"] == lost
    assert world.traffic.grants.held_by("pode2") == held, "its zones stay held"
    assert "GOAL_STALLED" not in codes and "GOAL_RETRY" not in codes, "nothing else settles it"
    assert not isolations(world)
    world.run(1.0)
    assert [i.reason for i in isolations(world)] == ["ROBOT_TIMEOUT"]
    assert "ROBOT_ISOLATED" in world.codes("pode2")
    assert world.result_of(call) is not None and not world.result_of(call).success


def test_a_lost_result_is_done_once_the_robot_reports_idle_at_the_end():
    world, call = sent_once()
    lost = lose(world, last_goal(world))
    world.run(3.0)
    assert moves(world) == 1, "MOVING settles nothing"
    stop_quietly(world, at="N_W")
    world.step()
    view = world.view("pode2")
    assert view["node"] == "N_W" and view["attempts"] == 0
    assert view["goal_id"] != lost and moves(world) == 2, "the next leg goes out"
    assert "GOAL_ENDED_AT_TARGET" in world.codes("pode2")
    world.run(world.params.robot_timeout_settle_s)
    assert not isolations(world, "ROBOT_TIMEOUT"), "settled once"


def test_a_lost_result_is_a_failed_attempt_once_the_robot_reports_idle_elsewhere():
    world, call = sent_once()
    p = world.params
    lost = lose(world, last_goal(world))
    stop_quietly(world)
    world.step()
    view = world.view("pode2")
    assert view["goal_id"] is None and view["attempts"] == 1
    assert world.codes("pode2")[-1] == "GOAL_RETRY"
    world.run(p.move_retry_wait_s + 0.5)
    assert moves(world) == 2 and last_goal(world) != lost, "the retry flow sends it again"
    world.run(p.robot_timeout_settle_s)
    assert world.codes("pode2").count("ROBOT_TIMEOUT") == 1
    assert not isolations(world, "ROBOT_TIMEOUT"), "settled once"


def test_a_late_result_inside_the_settle_time_settles_it_like_any_result():
    world, call = sent_once()
    lost = lose(world, last_goal(world))
    world.run(5.0)
    world.traffic.on_move_result("pode2", lost, False, "NAV_ABORTED")
    assert world.view("pode2")["goal_id"] is None and world.view("pode2")["attempts"] == 1
    world.run(world.params.robot_timeout_settle_s)
    assert not isolations(world, "ROBOT_TIMEOUT")


def test_a_lost_dock_result_docks_again_once_the_robot_reports_idle():
    """도킹 뒤에는 성공·실패 모두 IDLE이고 자리도 같다. 재시도로 다시 도킹해 정한다."""
    world, call = sent_once(speed=0.12)
    assert world.run_until(lambda: world.sent(DockCmd, "pode2"), 300.0)
    dock = world.sent(DockCmd, "pode2")[-1]
    reports(world, "pode2", "DOCKING")
    lose(world, dock.goal_id)
    world.bots["pode2"].dock = None  # 도킹은 끝났지만 결과를 잃었다
    world.run(3.0)
    assert world.result_of(call) is None and world.view("pode2")["goal_id"] == dock.goal_id
    world.__dict__.pop("_state")
    world.step()
    assert world.view("pode2")["goal_id"] is None and world.codes("pode2")[-1] == "GOAL_RETRY"
    assert world.run_until(lambda: world.result_of(call) is not None, 60.0)
    assert world.result_of(call).success and len(world.sent(DockCmd, "pode2")) == 2
    assert not isolations(world)
