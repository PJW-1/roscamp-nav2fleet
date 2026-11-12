"""Nav2 파라미터를 사용자 파일과 회피 담당 파일에서 모은다(추가 지시서 3의 7-1). ROS 없이 돈다.

- 사용자 파일(params/nav2_params.yaml): Nav2 파라미터 파일 그대로다. FollowPathGoal 컨트롤러의
  안쪽 값과 비전 관측원의 값은 없다(이름 FollowPathGoal은 controller_plugins에 있다).
- 회피 담당 파일(params/avoidance.yaml): 키가 둘뿐이다.
    follow_path_goal   controller_server의 FollowPathGoal 안에 그대로 들어간다
    vision_obstacles   두 costmap 장애물 층의 vision 관측원 안에 그대로 들어간다(켰을 때만)
  다른 키가 있으면 받지 않는다. 그 밖의 설정은 사용자의 것이다(docs/회피_약속.md).
"""
AVOIDANCE_KEYS = ("follow_path_goal", "vision_obstacles")
GOAL_CONTROLLER = "FollowPathGoal"
VISION_SOURCE = "vision"
COSTMAPS = ("local_costmap", "global_costmap")


def merge(base: dict, extra: dict) -> None:
    """extra를 base에 깊이 덮는다."""
    for key, value in extra.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merge(base[key], value)
        else:
            base[key] = value


def foreign_keys(avoidance: dict) -> list[str]:
    """회피 담당 파일에 들어 있는, 회피 담당의 것이 아닌 키."""
    return sorted(set(avoidance) - set(AVOIDANCE_KEYS))


def avoidance_values_in(user: dict) -> list[str]:
    """사용자 파일에 들어 있는 회피 담당의 값(있으면 안 된다: 값의 자리는 하나다)."""
    found = []
    if GOAL_CONTROLLER in user.get("controller_server", {}).get("ros__parameters", {}):
        found.append(f"controller_server.{GOAL_CONTROLLER}")
    for name in COSTMAPS:
        if VISION_SOURCE in _obstacle_layer(user, name):
            found.append(f"{name}.obstacle_layer.{VISION_SOURCE}")
    return found


def with_avoidance(data: dict, avoidance: dict, vision: bool = False) -> dict:
    """data(사용자 파일의 내용)에 회피 담당의 값을 넣는다. data를 바꾸고 돌려준다.

    FollowPathGoal이 data에 이미 있으면 그대로 둔다: 시뮬레이션이 속도 상한을 건 파일이다
    (fleet_gz_sim.launch_parts.sim_nav_params가 같은 회피 담당 파일에서 먼저 넣는다).
    """
    foreign = foreign_keys(avoidance)
    if foreign:
        raise ValueError(
            f"the avoidance params file may hold only {', '.join(AVOIDANCE_KEYS)}. "
            f"These settings are the user's (nav2_params.yaml): {', '.join(foreign)}")
    controllers = data["controller_server"]["ros__parameters"]
    if GOAL_CONTROLLER not in controllers:
        controllers[GOAL_CONTROLLER] = dict(avoidance["follow_path_goal"])
    if vision:
        for name in COSTMAPS:
            layer = _obstacle_layer(data, name)
            layer["observation_sources"] = f"{layer['observation_sources']} {VISION_SOURCE}"
            layer[VISION_SOURCE] = dict(avoidance["vision_obstacles"])
    return data


def _obstacle_layer(data: dict, costmap: str) -> dict:
    node = data.get(costmap, {}).get(costmap, {})
    return node.get("ros__parameters", {}).get("obstacle_layer", {})
