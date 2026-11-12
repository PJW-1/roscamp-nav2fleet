"""comm_loss_policy: 로봇이 끊김에서 어떻게 서든 관제는 끊긴 로봇의 base 구역을 쥔다."""
from dataclasses import replace

import pytest
from fleet_core.common.params import load_params
from test_zone_move_orders import ON, Harness


def test_the_default_is_stop_and_the_policy_is_in_the_params_file():
    assert load_params().comm_loss_policy == "stop"


@pytest.mark.parametrize("policy", ["", "Stop", "base-end", "go_on"])
def test_an_unknown_policy_is_refused_at_start_up(policy):
    with pytest.raises(ValueError, match="comm_loss_policy"):
        replace(load_params(), comm_loss_policy=policy)


def base_until_under(policy):
    world = Harness(params=replace(ON, comm_loss_policy=policy))
    world.step()
    world.call("WORK")
    world.run(2.0)
    world.silent.add("pinky2")
    assert world.run_until(lambda: world.view("pinky2")["state"] == "COMM_LOST", 10.0)
    return world.traffic._robots["pinky2"].base_until


def test_the_fleet_holds_the_base_of_a_silent_robot_under_both_policies():
    """Holding longer is never unsafe, so a robot and a fleet that disagree stay safe."""
    assert base_until_under("stop") == base_until_under("base_end")
