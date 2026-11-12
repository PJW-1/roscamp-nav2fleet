"""넓은 구역 정원의 합이 로봇 수보다 작으면 시작할 때 한 번 알린다(추가 지시서 2의 5절)."""
from zone_harness import Harness


def warnings(world):
    return [(e.level, e.robot, e.detail) for e in world.events if e.code == "ZONE_CAPACITY_LOW"]


def test_fewer_shared_places_than_robots_is_warned_once_at_start():
    world = Harness(starts={"pinky1": "P-1", "pinky2": "P-2", "pinky3": "P-3", "pinky4": "W_Q"})
    world.step()
    assert warnings(world) == [("WARN", "", "shared zones hold 3 robots in all (HALL 3) but "
                                            "4 robots run: they can wait on each other for good")]
    world.run(5.0)
    assert len(warnings(world)) == 1


def test_enough_shared_places_say_nothing():
    for layout in ("c", "d", "d2"):
        world = Harness(layout=layout)
        world.run(1.0)
        assert warnings(world) == [], layout
