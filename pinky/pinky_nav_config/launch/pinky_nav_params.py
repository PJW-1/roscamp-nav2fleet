# ---- Nav2 파라미터 딕셔너리에 회피·피어 설정을 합치는 도우미 ----
import copy

AVOIDANCE_KEYS = ("follow_path_goal", "vision_obstacles")
GOAL_CONTROLLER = "FollowPathGoal"
VISION_SOURCE = "vision"
COSTMAPS = ("local_costmap", "global_costmap")
PEER_LAYER = "peer_layer"
INFLATION_LAYER = "inflation_layer"


# ---- 딕셔너리 병합과 회피 키 검사 ----
def merge(base: dict, extra: dict) -> None:
    for key, value in extra.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merge(base[key], value)
        else:
            base[key] = value


def foreign_keys(avoidance: dict) -> list[str]:
    return sorted(set(avoidance) - set(AVOIDANCE_KEYS))


def avoidance_values_in(user: dict) -> list[str]:
    found = []
    if GOAL_CONTROLLER in user.get("controller_server", {}).get("ros__parameters", {}):
        found.append(f"controller_server.{GOAL_CONTROLLER}")
    for name in COSTMAPS:
        if VISION_SOURCE in _obstacle_layer(user, name):
            found.append(f"{name}.obstacle_layer.{VISION_SOURCE}")
    return found


# ---- 회피와 피어 레이어를 파라미터에 적용 ----
def with_avoidance(data: dict, avoidance: dict, vision: bool = False) -> dict:
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


def with_peers(data: dict, overlay: dict) -> dict:
    for name in COSTMAPS:
        node = data[name][name]["ros__parameters"]
        plugins = list(node["plugins"])
        if PEER_LAYER not in plugins:
            plugins.insert(plugins.index(INFLATION_LAYER), PEER_LAYER)
        node["plugins"] = plugins
        node[PEER_LAYER] = copy.deepcopy(overlay[PEER_LAYER])
    return data


def _obstacle_layer(data: dict, costmap: str) -> dict:
    node = data.get(costmap, {}).get(costmap, {})
    return node.get("ros__parameters", {}).get("obstacle_layer", {})
