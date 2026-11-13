from __future__ import annotations

from fleet_core.common.commands import AssignmentView, RobotStatusView, ZoneView
from fleet_core.orders.orchestrator import FILLING, SWAPPING
from fleet_core.traffic.mission import AT_STATION, DOCKING, OUT_OF_SERVICE, TO_OUTBOUND, TO_WORK


class ManagerViews:
    def zones_view(self) -> list[ZoneView]:
        return self.traffic.zones_view()

    def assignments_view(self) -> list[AssignmentView]:
        views = []
        for robot in self.traffic.robots:
            t = self.traffic.robot_view(robot)
            if not t["on_map"] or t["fault"] == OUT_OF_SERVICE:
                continue
            order_id = self.orchestrator.order_of(robot)
            phase, dest = t["phase"], t["destination"]
            docking = dest if phase == DOCKING else None
            if phase == AT_STATION and dest == self._work:
                views.append(AssignmentView(robot, order_id, self._work, "FILL"))
            elif phase == AT_STATION:
                views.append(AssignmentView(robot, order_id, self._outbound, "SWAP"))
            elif phase == TO_OUTBOUND or docking == self._outbound:
                views.append(AssignmentView(robot, order_id, self._outbound, "TO_OUTBOUND"))
            elif docking == self._work or (phase == TO_WORK and t["called"]):
                views.append(AssignmentView(robot, order_id, self._work, "TO_WORK"))
            else:
                views.append(AssignmentView(robot, order_id, "", "WAIT"))
        return views

    def robots_view(self) -> list[RobotStatusView]:
        tasks = {view.robot_id: view for view in self.assignments_view()}
        views = []
        for robot in self.traffic.robots:
            task = tasks.get(robot)
            views.append(RobotStatusView(
                robot_id=robot,
                task_type=task.task_type if task else "",
                order_id=(task.order_id or "") if task else "",
                **self.traffic.status_view(robot)))
        return views

    def robot_view(self, robot) -> dict:
        t = self.traffic.robot_view(robot)
        o = self.orchestrator.robot_view(robot)
        if not t or not o:
            return {}
        phase = t["phase"]
        if phase == AT_STATION:
            phase = FILLING if t["destination"] == self._work else SWAPPING
        return {
            "state": t["fault"] or t["comm"] or phase,
            "phase": phase,
            "basket": o["basket"],
            "order": o["order"],
            "placed": o["placed"],
            "called": t["called"],
            "held": t["held"],
            "path": t["path"],
            "waypoint": t["waypoint"],
            "destination": t["destination"],
            "goal_id": t["goal_id"],
            "attempts": t["attempts"] + self.arm.attempts(robot),
            "blocked_lane": t["blocked_lane"],
        }
