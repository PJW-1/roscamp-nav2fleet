"""구역 검사: 구역을 놓을 때의 거리와 레인 밖 놓음, 구역을 쥐지 않고 선 로봇, 로봇 위치."""
from __future__ import annotations

from fleet_core.fleet_map import SHARED

EPS = 1e-9


class ZoneChecks:
    def _zones_at(self, x: float, y: float) -> list[str]:
        """이 점을 품은 구역들(경계 위면 둘 이상)."""
        return [z for z in self.site.zones if self.site.distance_to_zone(x, y, z) <= EPS]

    def _outside_m(self, x: float, y: float) -> float:
        """가장 가까운 구역까지(어느 구역 안이면 0)."""
        return min(self.site.distance_to_zone(x, y, z) for z in self.site.zones)


    def _position(self, robot: str) -> tuple[float, float] | None:
        if robot in self.truth:
            return self.truth[robot]
        pose = self.poses.get(robot)
        return (pose[0], pose[1]) if pose else None

    def _positions(self) -> dict[str, tuple[float, float]]:
        return {robot: self._position(robot) for robot in {*self.poses, *self.truth}}

    def off_course(self, robot: str) -> bool:
        """로봇이 모든 구역에서 벗어난 것이 확인되면 True."""
        position = self._position(robot)
        if position is None:
            return False
        return self._outside_m(*position) > self.margin_m

    def held_by(self, robot: str) -> list[str]:
        return sorted(z for z, holders in self.holders.items() if robot in holders)

    def _check_release(self, t: float, zone: str, robot: str) -> None:
        self._check_off_lane_release(t, zone, robot)
        position = self._position(robot)
        if position is None:
            self._violate(t, f"{zone} released from {robot} with no known pose")
            self.release_violations += 1
            return
        cfg = self.scenario.robots.get(robot)
        noise = cfg.pose_noise_m if cfg else 0.0
        dist = self.site.distance_to_zone(position[0], position[1], zone)
        report = self.poses.get(robot)
        reported = self.site.distance_to_zone(report[0], report[1], zone) if report else ""
        self.release_rows.append((t, zone, robot, dist, reported))
        # 매니저는 잡음 낀 보고로 판단하고 여유 거리는 그 잡음을 흡수하려는 것이다.
        # 그래서 실제 위치는 몸이 구역을 벗어났음만 보이면 된다.
        limit = self.radius_m if robot in self.truth else max(0.0, self.margin_m - 2 * noise)
        if dist <= limit:
            self._violate(t, f"{zone} released from {robot} while {dist:.2f} m from it")
            self.release_violations += 1

    def _check_off_lane_release(self, t: float, zone: str, robot: str) -> None:
        """보고가 레인 밖인 동안에는 실제 위치를 모르므로 어떤 구역도 놓으면 안 된다."""
        if any(start <= t < end for start, end in self._drifts.get(robot, ())):
            self.off_lane_releases += 1
            self._violate(t, f"{zone} released from {robot} while its reports were off the lane")

    def _violate(self, t: float, text: str) -> None:
        self.violations.append(f"t={t:.1f} {text}")


    def _put_back_spot(self, robot: str, x: float, y: float):
        """놓인 지점에 아직 서 있으면 (지점, 놓은 시각, 유예 끝), 떠났으면 지우고 None."""
        spot = self._put_back.get(robot)
        if spot is not None and self.site.nearest_node(x, y) != spot[0]:
            del self._put_back[robot]
            return None
        return spot


    def _still_on_start_cell(self, robot: str, x: float, y: float) -> bool:
        """매니저는 모든 로봇의 보고를 들은 뒤에야 첫 구역을 잡는데, speed_factor 5에서는
        유예 시간을 넘길 수 있다. 아직 로봇을 움직인 것은 없다."""
        cfg = self.scenario.robots.get(robot)
        if robot in self._left_start or cfg is None:
            return False
        if self.site.nearest_node(x, y) == cfg.start:
            return True
        self._left_start.add(robot)
        return False

    def _check_unheld(self, t: float) -> None:
        current = set()
        for robot, (x, y) in self._positions().items():
            spot = self._put_back_spot(robot, x, y)
            if spot is None and self._still_on_start_cell(robot, x, y):
                continue
            inside = self._zones_at(x, y)
            held = any(robot in self.holders.get(z, ()) for z in inside)
            if held and spot is not None:
                del self._put_back[robot]  # 잡았다
            if not inside or held:
                continue
            zone = inside[0]
            if spot is not None and t < spot[2]:
                continue
            spec = self.site.zones[zone]
            if spec.kind == SHARED and len(self.holders.get(zone, ())) >= spec.capacity:
                continue  # 교통 층은 자리가 나면 잡는다(Nav2 goal이 가로지른 공유 구역)
            key = (robot, zone)
            current.add(key)
            # 놓인 뒤 계속 못 잡았으면 놓은 시각부터 잰다
            since = self._unheld_since.setdefault(key, spot[1] if spot is not None else t)
            if t - since >= self.grace_s and key not in self._unheld_counted:
                self._unheld_counted.add(key)
                self.unheld_violations += 1
                holders = ",".join(sorted(self.holders.get(zone, ()))) or "nobody"
                self._violate(t, f"{robot} is in {zone} but the holders are {holders}")
        for key in list(self._unheld_since):
            if key not in current:
                del self._unheld_since[key]
                self._unheld_counted.discard(key)
