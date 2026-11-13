"""지점이 없는 넓은 구역(배치 C와 D의 통로)의 한가운데에 선 로봇.

Gazebo 3대 경기(배치 C)에서 나왔다: 북쪽 통로를 가던 로봇이 막혀 서자 관제는 그 연결을 막힘으로
표시했고, 그 뒤로는 통로 안에 지점이 없어서 로봇에게 갈 길을 찾지 못했다. 로봇은 경기가 끝날
때까지 goal 없이 서 있었고 사건도 남지 않았다.
"""
from zone_harness import Harness

BOX = (1.20, 0.75, 0.10)  # 북쪽 통로 한가운데의 상자


def stopped_in_the_corridor():
    """할 일 없는 pinky2가 O_Q에서 하나 남은 주차 자리 P-2로 가다 북쪽 통로의 상자 앞에 섰다."""
    world = Harness(layout="c", starts={"pinky1": "P-1", "pinky2": "O_Q", "pinky3": "P-3"})
    world.obstacles.append(BOX)
    world.step()
    assert world.run_until(lambda: "ZONE_MARKED_BLOCKED" in world.codes("pinky2"), 60.0)
    bot = world.bots["pinky2"]
    assert world.site.zone_at(bot.x, bot.y) == "CORR_N", "it stands inside the corridor"
    return world


def test_a_robot_stopped_inside_a_corridor_gets_another_way():
    world = stopped_in_the_corridor()
    assert world.view("pinky2")["goal_id"], "a new goal goes out from where it stands"
    # 막힌 연결의 끝(P-2)으로 선 자리에서 곧장 다시 가지 않는다: 같은 것에 또 막힌다
    assert world.moves("pinky2")[-1].target_id in ("P-3", "N_W", "N_E")
    parked = world.run_until(lambda: world.view("pinky2")["state"] == "PARKED", 300.0)
    assert parked, world.log[-12:]
    assert world.view("pinky2")["node"] == "P-2", "it parks by the south way or the express"
    assert "GOAL_FAILED" not in world.codes() and not world.problems


def test_a_robot_with_no_way_left_is_reported_as_idle():
    world = stopped_in_the_corridor()
    p = world.params
    world.traffic._reach = lambda zone: {}  # 이제부터는 갈 수 있는 지점이 하나도 없다
    # 열려 있던 goal이 한 번 더 막히고, 그 표시가 풀린 뒤에야 "기다림"이 끝난다
    limit = p.blocked_mark_s + p.blocked_retry_s + p.idle_watch_s + 5.0
    idle = world.run_until(lambda: "ROBOT_IDLE" in world.codes("pinky2"), limit)
    assert idle, "a robot that stands with nowhere to go is not silent"
    assert world.traffic._robots["pinky2"].wait_for == (), "it is not shown as waiting any more"
