"""구역 세계 시험 도우미: 관제 대역(RecordingManager)과 흠 없는 세계."""
from fake_robots.world import Imperfections, RecordingManager, ZoneWorld
from fleet_core.common.commands import MoveCmd


def quiet_world(layout="d2", robots=None, injections=(), imperfect=None, **kw):
    """흠 없는 세계. 관제는 대역이라 시험이 명령을 직접 넣는다(world.send)."""
    return ZoneWorld(layout, seed=1, robots=robots or {"pinky1": "P-1"}, orders=[],
                     injections=injections,
                     imperfect=Imperfections.none() if imperfect is None else imperfect,
                     manager_factory=RecordingManager, **kw)


def place(world, name, x, y, yaw=0.0):
    r = world.robots[name]
    r.x, r.y, r.yaw = x, y, yaw


def nav(world, robot, goal_id, node=None, xy=None, yaw=0.0):
    x, y = world.site.node_pose(node) if node else xy
    return MoveCmd(robot, goal_id, node or "", x, y, yaw, mode="nav_goal")


def follow(world, robot, goal_id, nodes, yaw=0.0, final=True):
    points = tuple((*world.site.node_pose(n), yaw) for n in nodes)
    x, y = world.site.node_pose(nodes[-1])
    return MoveCmd(robot, goal_id, nodes[-1], x, y, yaw, path=points, final_goal=final)


def results(world, method, robot="pinky1"):
    """(시각, goal_id, 성공, 이유)."""
    return [(t, args[1], args[2], args[3] if len(args) > 3 else "")
            for t, args in world.manager.of(method, robot)]


def states(world, robot="pinky1"):
    return [(t, args[4]) for t, args in world.manager.of("on_robot_state", robot)]
