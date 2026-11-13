# ---- 주문 메시지 변환, 지도 구독, 주문 경로 마커 발행 ----
from __future__ import annotations

from geometry_msgs.msg import Point as PointMsg
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from pinky_fsm.costmap import Grid
from pinky_fsm.geometry import quaternion_yaw
from pinky_fsm.order_check import segment_problem
from pinky_fsm.order_markers import dashes, order_lines
from pinky_fsm.order_queue import MODE_NAV_GOAL, OrderGoal, Seg

# ---- 마커 QoS, 색, 선 굵기 상수 ----
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
BASE_COLOR = ColorRGBA(r=0.1, g=0.8, b=0.2, a=1.0)
HORIZON_COLOR = ColorRGBA(r=1.0, g=0.6, b=0.0, a=1.0)
LINE_WIDTH_M = 0.02


# ---- ROS 메시지를 구간, 주문, 격자로 바꾸기 ----
def _yaw(q) -> float:
    return quaternion_yaw(q.x, q.y, q.z, q.w)


def segment(msg) -> Seg:
    path = tuple((p.pose.position.x, p.pose.position.y, _yaw(p.pose.orientation))
                 for p in msg.path.poses)
    t = msg.target_pose.pose
    target = ((t.position.x, t.position.y, _yaw(t.orientation))
              if msg.mode == MODE_NAV_GOAL else None)
    return Seg(int(msg.seq_id), msg.mode, msg.controller_id, path, target, msg.end_node_id,
               tuple(msg.zone_ids))


def order_goal(req) -> OrderGoal | None:
    if not req.order_id:
        return None
    return OrderGoal(req.order_id, int(req.order_update_id), req.stitch_node_id,
                     tuple(segment(s) for s in req.base), tuple(segment(s) for s in req.horizon))


def _grid(msg: OccupancyGrid) -> Grid:
    o = msg.info.origin
    return Grid(msg.info.resolution, o.position.x, o.position.y, _yaw(o.orientation),
                msg.info.width, msg.info.height, msg.data)


# ---- 지도와 keepout 구독, 구간 검사 ----
class OrderMaps:
    def __init__(self, node, map_topic: str, keepout_topic: str, callback_group=None):
        self.map: Grid | None = None
        self.keepout: Grid | None = None
        node.create_subscription(OccupancyGrid, map_topic, self._on_map, LATCHED,
                                 callback_group=callback_group)
        if keepout_topic:
            node.create_subscription(OccupancyGrid, keepout_topic, self._on_keepout, LATCHED,
                                     callback_group=callback_group)

    def _on_map(self, msg) -> None:
        self.map = _grid(msg)

    def _on_keepout(self, msg) -> None:
        self.keepout = _grid(msg)

    def check(self, seg: Seg) -> str:
        return segment_problem(seg, self.map, self.keepout)


# ---- 주문 경로를 RViz 마커로 발행 ----
class OrderMarkers:
    def __init__(self, node, topic: str, frame: str):
        self._node, self._frame = node, frame
        self._pub = node.create_publisher(MarkerArray, topic, LATCHED)
        self._last = None

    def publish(self, start, base, horizon) -> None:
        lines = order_lines(start, base, horizon)
        if lines == self._last:
            return
        self._last = lines
        stamp = self._node.get_clock().now().to_msg()
        out = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for k, ln in enumerate(lines):
            m = Marker(ns="horizon" if ln.horizon else "base", id=k, action=Marker.ADD)
            m.header.frame_id, m.header.stamp = self._frame, stamp
            m.pose.orientation.w = 1.0
            m.scale.x = LINE_WIDTH_M
            m.color = HORIZON_COLOR if ln.horizon else BASE_COLOR
            if ln.horizon:
                m.type = Marker.LINE_LIST
                pts = [p for pair in dashes(ln.points) for p in pair]
            else:
                m.type = Marker.LINE_STRIP
                pts = list(ln.points)
            m.points = [PointMsg(x=float(x), y=float(y), z=0.02) for x, y in pts]
            if len(m.points) >= 2:
                out.markers.append(m)
        self._pub.publish(out)
