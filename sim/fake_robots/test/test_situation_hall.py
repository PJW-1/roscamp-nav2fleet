"""상황 시험(지시서 6-2): 마당 장애물, 사람이 잠깐 지나감."""
import pytest
from situation_helpers import ALL_LAYOUTS, SEEDS, check


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("layout", ALL_LAYOUTS)
def test_robots_get_past_a_box_in_the_hall(layout, seed):
    check("hall_obstacle", layout, seed)


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("layout", ALL_LAYOUTS)
def test_a_person_passing_is_waited_for_without_detour_or_retreat(layout, seed):
    check("person_passing", layout, seed)
