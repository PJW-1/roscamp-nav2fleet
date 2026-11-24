"""Isaac Lab(Newton) 매장 아레나: goal-reach env 에 플릿 맵의 벽과 선반을 더한다.

박스는 env 생성 때 arena_spec.arena_boxes 로 만들어 모든 env 에 복제된다. 에피소드마다 경로 하나
(route_spec.build_routes)를 달리며, 관측 목표는 경로상 lookahead_m 앞 점이다.
장애물/mover/로봇은 kinematic 박스로 리셋이나 제어 스텝마다 옮기고, 안 쓰는 것은 남쪽 벽 뒤에 둔다.
"""

from __future__ import annotations

import os
import sys

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.utils import configclass
from isaaclab_physx.sim.schemas import PhysxRigidBodyCfg

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "goal_reach"))

import arena_spec as arena  # noqa: E402
import mover_spec as ms  # noqa: E402
import numpy as np  # noqa: E402
import robot_spec as rbs  # noqa: E402
import route_spec as rs  # noqa: E402
import task_spec as ts  # noqa: E402
import torch  # noqa: E402
from goal_reach_env import GoalReachEnv, GoalReachEnvCfg  # noqa: E402

COLORS = {
    "wall": (0.55, 0.55, 0.6),
    "shelf": (0.75, 0.55, 0.3),
    "obstacle": (0.2, 0.45, 0.8),
    "mover": (0.85, 0.2, 0.25),
    "robot": (0.2, 0.7, 0.35),
}
# 충돌 원인 분류 (충돌 스텝에서 가장 가까운 것)
HIT_KINDS = ("wall_shelf", "box", "mover", "robot")
LIFT_M = 0.002  # 장애물 박스를 바닥에서 띄우는 높이 [m] (바닥 접촉 방지)


def box_asset_cfg(box: arena.Box) -> AssetBaseCfg:
    """각 env 안 맵 좌표의 정적 박스 콜라이더."""
    sx, sy = box.x1 - box.x0, box.y1 - box.y0
    return AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/arena_" + box.name,
        spawn=sim_utils.CuboidCfg(
            size=(sx, sy, box.height_m),
            collision_props=sim_utils.UsdPhysicsCollisionCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=COLORS[box.kind]),
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(box.x0 + 0.5 * sx, box.y0 + 0.5 * sy, 0.5 * box.height_m)
        ),
    )


def add_arena(scene_cfg, boxes: list[arena.Box]) -> None:
    """박스마다 씬 항목을 추가한다."""
    for box in boxes:
        setattr(scene_cfg, "arena_" + box.name, box_asset_cfg(box))


def park_xy(k: int) -> tuple[float, float]:
    """안 쓰는 장애물 슬롯 k 의 대기 위치: 남쪽 벽 뒤."""
    return (0.3 + 0.5 * k, -0.4)


def obstacle_cfg(k: int, side: float, height: float) -> RigidObjectCfg:
    """장애물 슬롯 k: 리셋 때 옮기는 kinematic 정사각 박스."""
    x, y = park_xy(k)
    return RigidObjectCfg(
        prim_path=f"{{ENV_REGEX_NS}}/obstacle_{k}",
        spawn=sim_utils.CuboidCfg(
            size=(side, side, height),
            rigid_props=[
                sim_utils.UsdPhysicsRigidBodyCfg(kinematic_enabled=True),
                PhysxRigidBodyCfg(disable_gravity=True),
            ],
            mass_props=sim_utils.MassCfg(mass=10.0),
            collision_props=sim_utils.UsdPhysicsCollisionCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=COLORS["obstacle"]),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(x, y, 0.5 * height + LIFT_M)),
    )


def mover_park_xy(i: int) -> tuple[float, float]:
    """비활성 mover 의 대기 위치."""
    return (1.6 + 0.3 * i, -0.4)


def mover_cfg(i: int, shape: ms.MoverShape) -> RigidObjectCfg:
    """mover i: 제어 스텝마다 옮기는 kinematic 원기둥 또는 박스."""
    common = dict(
        rigid_props=[
            sim_utils.UsdPhysicsRigidBodyCfg(kinematic_enabled=True),
            PhysxRigidBodyCfg(disable_gravity=True),
        ],
        mass_props=sim_utils.MassCfg(mass=10.0),
        collision_props=sim_utils.UsdPhysicsCollisionCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=COLORS["mover"]),
    )
    if shape.kind == "cylinder":
        spawn = sim_utils.CylinderCfg(radius=shape.size[0], height=shape.height, **common)
    else:
        spawn = sim_utils.CuboidCfg(size=shape.size, **common)
    x, y = mover_park_xy(i)
    return RigidObjectCfg(
        prim_path=f"{{ENV_REGEX_NS}}/mover_{shape.name}",
        spawn=spawn,
        init_state=RigidObjectCfg.InitialStateCfg(pos=(x, y, 0.5 * shape.height + LIFT_M)),
    )


def robot_park_xy(k: int) -> tuple[float, float]:
    """비활성 스크립트 로봇의 대기 위치."""
    return (0.3 + 0.5 * k, -0.9)


def robot_cfg(k: int) -> RigidObjectCfg:
    """스크립트 플릿 로봇 k: 스텝마다 옮기는 kinematic Pode 크기 박스."""
    sx, sy, sz = rbs.ROBOT.size
    x, y = robot_park_xy(k)
    return RigidObjectCfg(
        prim_path=f"{{ENV_REGEX_NS}}/fleet_robot_{k}",
        spawn=sim_utils.CuboidCfg(
            size=(sx, sy, sz),
            rigid_props=[
                sim_utils.UsdPhysicsRigidBodyCfg(kinematic_enabled=True),
                PhysxRigidBodyCfg(disable_gravity=True),
            ],
            mass_props=sim_utils.MassCfg(mass=10.0),
            collision_props=sim_utils.UsdPhysicsCollisionCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=COLORS["robot"]),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(x, y, 0.5 * sz + LIFT_M)),
    )


@configclass
class StoreArenaEnvCfg(GoalReachEnvCfg):
    # 덮어쓸 ArenaSpec 필드 (arena_spec.split_overrides)
    arena_overrides: dict = {}


class StoreArenaEnv(GoalReachEnv):
    cfg: StoreArenaEnvCfg

    def __init__(self, cfg: StoreArenaEnvCfg, render_mode: str | None = None, **kwargs):
        a = self.arena = arena.make_arena_spec(cfg.arena_overrides)
        self.boxes = arena.arena_boxes(a)
        add_arena(cfg.scene, self.boxes)
        static = np.array([(b.x0, b.y0, b.x1, b.y1) for b in self.boxes])
        site = arena.load_site(a)
        self.routes = r = rs.build_routes(a, site, static)
        self.routes_bounds = site.bounds()
        for k, side in enumerate(r.obstacle_sizes):
            setattr(cfg.scene, f"obstacle_{k}", obstacle_cfg(k, side, a.obstacle_height_m))
        for i, shape in enumerate(ms.MOVERS):
            setattr(cfg.scene, f"mover_{shape.name}", mover_cfg(i, shape))
        for k in range(int(a.robot_max)):
            setattr(cfg.scene, f"fleet_robot_{k}", robot_cfg(k))
        self.graph = rbs.node_graph(site)
        cfg.scene.env_spacing = a.env_spacing
        spec = ts.make_spec(cfg.spec_overrides)
        # 에피소드별 최고속도 (speed_cap_range): 시간은 가장 느린 상한에 맞춰 늘리고 env 마다 자기
        # 상한 길이(ep_limit)에서 끝남
        lo, hi = spec.speed_cap_range
        self.cap_varies = lo != hi
        self.cap_active = self.cap_varies or lo != ts.NAV_SPEED_MPS  # 아니면 ppo_k 규칙 그대로
        cfg.episode_length_s = a.episode_s * ts.NAV_SPEED_MPS / min(spec.speed_cap_range)
        # 크리틱 전용 항목: 움직이는 물체, 이어서 스크립트 로봇 칸 (task_spec.privileged_block)
        self.n_priv = len(ms.MOVERS) + int(a.robot_max)
        cfg.state_space = ts.PRIV_PER_OBJECT * self.n_priv if spec.critic_privileged else 0
        self._routes_ready = False
        super().__init__(cfg, render_mode, **kwargs)
        print(f"[ARENA] {a}")
        for b in self.boxes:
            print(
                f"[ARENA] {b.name}: x {b.x0:.3f}..{b.x1:.3f} y {b.y0:.3f}..{b.y1:.3f} "
                f"h {b.height_m:.2f}"
            )
        print(
            f"[ARENA] routes {r.count} = {len(r.pairs)} pairs x {r.layouts} layouts, nodes "
            f"{r.node_names}, path length {r.total.min():.2f}..{r.total.max():.2f} m"
        )
        dev, n = self.device, self.num_envs
        f32 = dict(device=dev, dtype=torch.float32)
        ends = np.array([r.start_goal(i) for i in range(r.count)])  # (R, 2, 2)
        self.r_path = torch.as_tensor(r.path, **f32)
        self.r_cum = torch.as_tensor(r.cum, **f32)
        self.r_npts = torch.as_tensor(r.npts, device=dev)
        self.r_start = torch.as_tensor(ends[:, 0], **f32)
        self.r_goal = torch.as_tensor(ends[:, 1], **f32)
        park = torch.tensor([park_xy(k) for k in range(len(r.obstacle_sizes))], **f32)
        box_xy = torch.as_tensor(r.obstacle_xy, **f32)
        self.r_obstacle = torch.where(box_xy.isnan(), park.expand_as(box_xy), box_xy)
        self.obstacles = [self.scene[f"obstacle_{k}"] for k in range(len(r.obstacle_sizes))]
        self.route = torch.zeros(n, dtype=torch.long, device=dev)
        self.prog = torch.zeros(n, dtype=torch.long, device=dev)
        self.la_dist = torch.zeros(n, device=dev)
        self.la_ang = torch.zeros(n, device=dev)
        self.d_goal = torch.zeros(n, device=dev)
        # mover: env 와 mover 마다 구간, 리셋 후 시간, 로봇 중심과 mover 표면의 최소 간격
        self.static_rects = torch.as_tensor(static, **f32)
        self.mover_objs = [self.scene[f"mover_{m.name}"] for m in ms.MOVERS]
        self.mover_legs = [
            {
                "a": torch.zeros(n, 2, **f32),
                "b": torch.zeros(n, 2, **f32),
                "speed": torch.zeros(n, **f32),
                "pause": torch.zeros(n, **f32),
                "phase": torch.zeros(n, **f32),
                "active": torch.zeros(n, dtype=torch.bool, device=dev),
            }
            for _ in ms.MOVERS
        ]
        self.mover_park = torch.tensor([mover_park_xy(i) for i in range(len(ms.MOVERS))], **f32)
        self.mover_t = torch.zeros(len(ms.MOVERS), n, **f32)  # mover 별 시계
        self.mover_held = torch.zeros(len(ms.MOVERS), n, dtype=torch.bool, device=dev)  # 대기 중
        self.mover_wait = torch.zeros(len(ms.MOVERS), n, **f32)  # 기다린 시간 [s]
        # mover 별 양보 거리
        self.mover_yield = torch.full((len(ms.MOVERS), n), float(a.mover_yield_m), **f32)
        # 벽 스침과 코너 시작
        self.wall_clear = torch.full((n,), float("inf"), **f32)
        self.graze_only = torch.zeros(n, dtype=torch.bool, device=dev)
        self.r_corner = torch.nonzero(
            arena.near_wall_starts(self.r_start, self.static_rects, a.corner_start_m)
        )[:, 0]
        self.corner_start = torch.zeros(n, dtype=torch.bool, device=dev)
        self.last_ep_corner = torch.zeros(n, dtype=torch.bool, device=dev)
        self._new_corner = torch.zeros(n, dtype=torch.bool, device=dev)
        # 인계 시작 (handover_start_prob): handover_nodes 에서 나가는 경로, env 별 초기 w
        names = set(a.handover_nodes.split(","))
        starts = [r.node_names[r.pairs[i // r.layouts][0]] for i in range(r.count)]
        self.r_handover = torch.tensor(
            [i for i, s_ in enumerate(starts) if s_ in names], device=dev, dtype=torch.long
        )
        self.handover_start = torch.zeros(n, dtype=torch.bool, device=dev)
        self.last_ep_handover = torch.zeros(n, dtype=torch.bool, device=dev)
        self._new_handover = torch.zeros(n, dtype=torch.bool, device=dev)
        self._new_w0 = torch.zeros(n, **f32)
        self.mover_gap = torch.full((n,), float("inf"), **f32)
        self.last_ep_mover_gap = torch.full((n,), float("inf"), **f32)
        # 스크립트 플릿 로봇
        g = self.graph
        self.g_t = rbs.graph_tensors(g, dev)
        # 예약 규칙용: 우리 경로가 그래프 노드 robot_reserve_m 이내를 지나는 호장 (inf: 안 지남)
        d = np.linalg.norm(r.path[:, :, None] - g.xy[None, None], axis=-1)  # (R, L, K)
        d[np.arange(r.path.shape[1])[None, :] >= r.npts[:, None]] = np.inf
        near = d.min(1) < a.robot_reserve_m
        node_arc = np.take_along_axis(r.cum, d.argmin(1), 1)  # (R, K)
        self.r_node_arc = torch.as_tensor(np.where(near, node_arc, np.inf), **f32)
        gi = {name: i for i, name in enumerate(g.names)}
        # 경로별 시작/목표 노드의 그래프 인덱스 (-1: 그래프 노드 아님)
        self.r_sg_node = torch.tensor(
            [[gi.get(r.node_names[s], -1), gi.get(r.node_names[e], -1)] for s, e in r.pairs],
            device=dev,
        ).repeat_interleave(r.layouts, 0)  # (R, 2)
        nk = int(a.robot_max)
        self.robot_objs = [self.scene[f"fleet_robot_{k}"] for k in range(nk)]
        self.robot_park = torch.tensor([robot_park_xy(k) for k in range(nk)], **f32)
        lay = rbs.chain_route(
            self.g_t,
            torch.zeros(n, int(a.robot_routes) + 1, dtype=torch.long, device=dev),
            torch.ones(n, device=dev),
            torch.zeros(n, int(a.robot_routes), device=dev),
        )
        self.robot_routes = [{k_: v.clone() for k_, v in lay.items()} for _ in range(nk)]
        self.robot_active = torch.zeros(nk, n, dtype=torch.bool, device=dev)
        self.robot_t = torch.zeros(nk, n, **f32)
        self.robot_wait = torch.zeros(nk, n, **f32)
        self.robot_back = torch.zeros(nk, n, **f32)
        self.robot_priority = torch.zeros(nk, n, dtype=torch.bool, device=dev)
        self.robot_gap_min = torch.full((n,), float("inf"), **f32)
        self.last_ep_robot_gap = torch.full((n,), float("inf"), **f32)
        self.hit_kind = torch.zeros(n, dtype=torch.long, device=dev)
        self.last_ep_hit_kind = torch.zeros(n, dtype=torch.long, device=dev)
        # 레시피 블록 상태: 에피소드 최고속도·시간 한도, 직전 목표, w 부호 반전, lidar_history
        # 프레임, 직전 스텝 로봇 위치
        # w 반전 벌점은 목표에, nav_bench 지표는 명령에 매김
        s = self.spec
        self.v_cap = torch.full((n,), float(min(s.speed_cap_range)), **f32)
        self.ep_limit = torch.full((n,), self.max_episode_length, device=dev, dtype=torch.long)
        self.target = torch.zeros(n, 2, **f32)
        self.prev_target = torch.zeros(n, 2, **f32)
        self.target_sign = torch.zeros(n, **f32)
        self.target_flipped = torch.zeros(n, dtype=torch.bool, device=dev)
        self.cmd_sign = torch.zeros(n, **f32)
        self.cmd_flips = torch.zeros(n, **f32)
        self.last_ep_cmd_flips = torch.zeros(n, **f32)
        self.last_ep_cap = torch.zeros(n, **f32)
        hist = max(s.lidar_frames - 1, 0) * s.lidar_stride_steps + 1
        self.lid_buf = torch.zeros(n, hist, s.lidar_sectors, **f32)
        self.lid_fresh = torch.ones(n, dtype=torch.bool, device=dev)
        self.robot_prev_xy = torch.zeros(n, nk, 2, **f32)
        self._routes_ready = True

    def sample_start(self, n: int) -> tuple[torch.Tensor, torch.Tensor]:
        """무작위 경로의 시작 노드와 yaw 를 뽑는다 (_reset_idx 가 이어 씀)."""
        yaw = (2 * torch.rand(n, device=self.device) - 1) * torch.pi
        if not self._routes_ready:  # super().__init__ 안의 리셋: 고정 시작점, 경로 없음
            xy = torch.tensor((self.arena.start_x, self.arena.start_y), device=self.device)
            return xy.expand(n, 2).clone(), yaw
        self._new_route = torch.randint(0, self.routes.count, (n,), device=self.device)
        p = self.arena.corner_start_prob
        self._new_corner = torch.zeros(n, dtype=torch.bool, device=self.device)
        if p > 0 and len(self.r_corner):  # 아니면 난수를 안 뽑아 이전 아레나와 같게 유지
            self._new_corner = torch.rand(n, device=self.device) < p
            pick = self.r_corner[torch.randint(0, len(self.r_corner), (n,), device=self.device)]
            self._new_route = torch.where(self._new_corner, pick, self._new_route)
            jit = (2 * torch.rand(n, device=self.device) - 1) * self.arena.corner_yaw_jitter
            away = arena.away_heading(self.r_path[self._new_route], jit)
            yaw = torch.where(self._new_corner, away, yaw)
        q = self.arena.handover_start_prob
        self._new_handover = torch.zeros(n, dtype=torch.bool, device=self.device)
        self._new_w0 = torch.zeros(n, device=self.device)
        if q > 0 and len(self.r_handover):  # 아니면 뽑지 않음: 이전 아레나 그대로
            ho = torch.rand(n, device=self.device) < q
            pick = self.r_handover[torch.randint(0, len(self.r_handover), (n,), device=self.device)]
            self._new_route = torch.where(ho, pick, self._new_route)
            self._new_corner = self._new_corner & ~ho
            ho_yaw, w0 = arena.handover_start(
                arena.path_heading(self.r_path[self._new_route]),
                torch.rand(n, 4, device=self.device),
                self.arena,
            )
            yaw = torch.where(ho, ho_yaw, yaw)
            self._new_handover = ho
            self._new_w0 = torch.where(ho, w0, 0.0)
        self._new_yaw = yaw
        return self.r_start[self._new_route], yaw

    def _follow(self, ids, xy, yaw) -> None:
        """경로 진행, 남은 길이, lookahead 목표와 목표 거리를 갱신한다."""
        r = self.route[ids]
        self.prog[ids], self.d_now[ids], la = rs.follow(
            self.r_path[r],
            self.r_cum[r],
            self.r_npts[r],
            self.prog[ids],
            xy,
            self.arena.lookahead_m,
            int(self.arena.follow_window),
        )
        self.la_dist[ids], self.la_ang[ids] = ts.goal_in_robot_frame(xy, yaw, la)
        self.d_goal[ids] = (self.goal_xy[ids] - xy).norm(dim=-1)

    def _reset_idx(self, env_ids):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        if self._routes_ready:
            self.last_ep_cmd_flips[env_ids] = self.cmd_flips[env_ids]
            self.last_ep_cap[env_ids] = self.v_cap[env_ids]
            self.last_ep_mover_gap[env_ids] = self.mover_gap[env_ids]
            self.last_ep_robot_gap[env_ids] = self.robot_gap_min[env_ids]
            self.last_ep_hit_kind[env_ids] = self.hit_kind[env_ids]
            self.last_ep_corner[env_ids] = self.corner_start[env_ids]
            self.last_ep_handover[env_ids] = self.handover_start[env_ids]
        super()._reset_idx(env_ids)
        if not self._routes_ready:
            return
        self.corner_start[env_ids] = self._new_corner
        self.handover_start[env_ids] = self._new_handover
        if self._new_handover.any():
            self._spin_up(env_ids, self._new_w0)
        s = self.spec
        if self.cap_varies:  # 아니면 뽑지 않음: 고정 상한 실행의 난수 순서 그대로
            self.v_cap[env_ids] = ts.sample_speed_cap(len(env_ids), self.device, s)
        if self.cap_active:
            cap = self.v_cap[env_ids]
            steps = torch.ceil(self.arena.episode_s * ts.NAV_SPEED_MPS / cap / self.step_dt)
            self.ep_limit[env_ids] = steps.long().clamp(max=self.max_episode_length)
        self.target[env_ids] = 0.0
        self.prev_target[env_ids] = 0.0
        self.target_sign[env_ids] = 0.0
        self.cmd_sign[env_ids] = 0.0
        self.cmd_flips[env_ids] = 0.0
        self.lid_fresh[env_ids] = True
        r = self._new_route
        self.route[env_ids] = r
        self.goal_xy[env_ids] = self.r_goal[r]
        self.prog[env_ids] = 0
        origins = self.scene.env_origins[env_ids]
        zero_vel = torch.zeros(len(env_ids), 6, device=self.device)
        for k, obj in enumerate(self.obstacles):
            pose = obj.data.default_root_pose.torch[env_ids].clone()
            pose[:, :2] = origins[:, :2] + self.r_obstacle[r, k]
            obj.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
            obj.write_root_velocity_to_sim_index(root_velocity=zero_vel, env_ids=env_ids)
        self._follow(env_ids, self.start_xy[env_ids], self._new_yaw)
        self.d_prev[env_ids] = self.d_now[env_ids]
        self._sample_movers(env_ids, r)
        self.mover_t[:, env_ids] = 0.0
        self.mover_held[:, env_ids] = False
        self.mover_wait[:, env_ids] = 0.0
        a = self.arena
        if a.mover_yield_mix:
            n_ = len(env_ids)
            self.mover_yield[:, env_ids] = ms.mixed_yield(
                n_, len(ms.MOVERS), a.mover_yield_m, device=self.device
            )
        elif a.mover_noyield_prob > 0:  # 아니면 난수를 안 뽑아 4b/4c 와 같게 유지
            noyield = torch.rand(len(env_ids), device=self.device) < a.mover_noyield_prob
            self.mover_yield[:, env_ids] = torch.where(noyield, 0.0, a.mover_yield_m)
        self._write_movers(env_ids)
        self.mover_gap[env_ids] = self.mover_gaps(self.start_xy[env_ids], env_ids)  # t = 0
        self._sample_robots(env_ids, r)
        self._write_robots(env_ids)
        self.robot_gap_min[env_ids] = self.robot_gaps(self.start_xy[env_ids], env_ids)
        for k, (pos, _, _) in enumerate(self.robot_state(env_ids)):
            self.robot_prev_xy[env_ids, k] = pos

    def _spin_up(self, env_ids, w0) -> None:
        """env_ids 를 w0 (n,) rad/s 로 제자리 회전하며 시작: 몸 yaw 속도, 바퀴 속도, 변화율 제한
        명령을 맞춤."""
        vel = torch.zeros(len(env_ids), 6, device=self.device)
        vel[:, 5] = w0
        self.robot.write_root_velocity_to_sim_index(root_velocity=vel, env_ids=env_ids)
        cmd = torch.stack((torch.zeros_like(w0), w0), -1)
        jv = torch.zeros_like(self.robot.data.default_joint_pos.torch[env_ids])
        jv[:, self.wheel_ids] = ts.twist_to_wheels(cmd)
        self.robot.write_joint_velocity_to_sim_index(velocity=jv, env_ids=env_ids)
        self.cmd[env_ids] = cmd

    def _sample_robots(self, env_ids, r) -> None:
        """env_ids 의 로봇 경로: 0..robot_max 대, 우리 시작/목표와 다른 노드에서 출발/정지."""
        a, n, dev = self.arena, len(env_ids), self.device
        nk = len(self.robot_objs)
        if nk == 0:  # 난수를 안 뽑아 4b 아레나의 난수열 유지
            return
        count = torch.randint(0, nk + 1, (n,), device=dev)
        banned = torch.zeros(n, len(self.graph.names), dtype=torch.bool, device=dev)
        sg = self.r_sg_node[r]
        for c in range(2):
            ok = sg[:, c] >= 0
            banned[ok.nonzero()[:, 0], sg[ok, c]] = True
        lo, hi = a.robot_speed_scale
        p0, p1 = a.robot_pause_s
        legs = int(a.robot_routes)
        ours = self.our_nodes(env_ids) if a.robot_reserve else None
        headon = bool(a.robot_headon) & (sg >= 0).all(-1)
        if a.robot_headon:
            count = torch.where(headon, count.clamp(min=1), count)
        for k in range(nk):
            chain = rbs.sample_chains(self.g_t, n, legs, banned, start_banned=ours)
            if k == 0 and a.robot_headon:  # 우리 목표 -> 우리 시작, 이후 임의
                chain[headon, 0], chain[headon, 1] = sg[headon, 1], sg[headon, 0]
                if legs > 1:
                    again = headon & (chain[:, 2] == sg[:, 0])
                    chain[again, 2] = sg[again, 1]
            speed = rbs.ROBOT.speed * (lo + torch.rand(n, device=dev) * (hi - lo))
            if self.cap_active:  # 스크립트 로봇은 에피소드 상한 x 자기 배율로 달림
                speed = speed * self.v_cap[env_ids] / ts.NAV_SPEED_MPS
            pause = p0 + torch.rand(n, legs, device=dev) * (p1 - p0)
            route = rbs.chain_route(self.g_t, chain, speed, pause)
            for key, v in route.items():
                self.robot_routes[k][key][env_ids] = v
            banned[torch.arange(n, device=dev), chain[:, 0]] = True
            self.robot_active[k, env_ids] = k < count
            self.robot_t[k, env_ids] = 0.0
            self.robot_wait[k, env_ids] = 0.0
            self.robot_back[k, env_ids] = 0.0
            if a.robot_priority_prob > 0:  # 아니면 난수를 안 뽑아 이전 실행과 같게 유지
                self.robot_priority[k, env_ids] = torch.rand(n, device=dev) < a.robot_priority_prob

    def our_nodes(self, ids=None) -> torch.Tensor:
        """(n, K) 우리 로봇이 잡고 있는 그래프 노드."""
        ids = slice(None) if ids is None else ids
        r = self.route[ids]
        done_arc = self.r_cum[r].gather(1, self.prog[ids][:, None])[:, 0]  # 지나온 경로
        return rbs.route_nodes_held(self.r_node_arc[r], done_arc, self.arena.robot_release_m)

    def blocked_nodes(self, k: int) -> torch.Tensor:
        """(n, K) 로봇 k 가 지나갈 수 없는 노드: 우리와 다른 로봇의 것."""
        held = self.our_nodes()
        n = self.num_envs
        rows = torch.arange(n, device=self.device)
        for j, route in enumerate(self.robot_routes):
            if j == k:
                continue
            act = self.robot_active[j]
            for node in rbs.robot_nodes(route, self.robot_t[j]):
                on = act & (node >= 0)
                held[rows[on], node[on]] = True
        return held

    def robot_state(self, ids=None):
        """로봇별 위치 (n, 2), yaw (n,), 활성 (n,); 비활성은 대기 위치."""
        ids = slice(None) if ids is None else ids
        out = []
        for k, route in enumerate(self.robot_routes):
            sub = {key: v[ids] for key, v in route.items()}
            pos, yaw, _ = rbs.robot_pose(sub, self.robot_t[k, ids])
            act = self.robot_active[k, ids]
            pos = torch.where(act[:, None], pos, self.robot_park[k].expand_as(pos))
            out.append((pos, yaw, act))
        return out

    def _write_robots(self, ids) -> None:
        origins = self.scene.env_origins[ids]
        for obj, (pos, yaw, _) in zip(self.robot_objs, self.robot_state(ids), strict=True):
            pose = obj.data.default_root_pose.torch[ids].clone()
            pose[:, :2] = origins[:, :2] + pos
            z = torch.zeros_like(yaw)
            pose[:, 3:7] = torch.stack((z, z, torch.sin(yaw / 2), torch.cos(yaw / 2)), -1)
            obj.write_root_pose_to_sim_index(root_pose=pose, env_ids=ids)

    def robot_gaps(self, xy: torch.Tensor, ids=None) -> torch.Tensor:
        """(n,) 우리 중심에서 가장 가까운 활성 로봇 박스까지 거리, 없으면 inf."""
        gap = torch.full((len(xy),), float("inf"), device=self.device)
        for pos, yaw, act in self.robot_state(ids):
            gap = torch.where(act, torch.minimum(gap, rbs.robot_gap(xy, pos, yaw)), gap)
        return gap

    def robot_intent(self, xy, yaw):
        """가까운 활성 로봇 순 intent: 경로점 (n, R, P, 2) 우리 프레임, 마스크, 우선권."""
        s = self.spec
        pts, dist, act = [], [], []
        for k, (pos, _, a_) in enumerate(self.robot_state()):
            p = rbs.intent_points(
                self.robot_routes[k], self.robot_t[k], s.intent_spacing_m, s.intent_points
            )
            pts.append(rbs.to_body(p, xy, yaw))
            dist.append(torch.where(a_, (pos - xy).norm(dim=-1), torch.full_like(xy[:, 0], 1e9)))
            act.append(a_)
        n, r = len(xy), s.intent_robots
        out_p = torch.zeros(n, r, s.intent_points, 2, device=self.device)
        out_m = torch.zeros(n, r, dtype=torch.bool, device=self.device)
        out_q = torch.zeros(n, r, dtype=torch.bool, device=self.device)
        if pts and not self.arena.intent_ablate:
            pts, dist, act = torch.stack(pts, 1), torch.stack(dist, 1), torch.stack(act, 1)
            order = dist.argsort(1)[:, :r]
            m = min(r, order.shape[1])
            rows = torch.arange(n, device=self.device)[:, None]
            out_p[:, :m] = pts[rows, order[:, :m]]
            out_m[:, :m] = act[rows, order[:, :m]]
            out_q[:, :m] = self.robot_priority.T[rows, order[:, :m]]
        return out_p, out_m, out_q

    def nearest_hit_kind(self, xy: torch.Tensor) -> torch.Tensor:
        """(n,) 가장 가까운 것의 HIT_KINDS 인덱스."""
        n = len(xy)
        d_static = ms.point_rect_distance(xy[:, None], self.static_rects.expand(n, -1, -1))[:, 0]
        sizes = torch.tensor(self.routes.obstacle_sizes, device=self.device)
        bxy = self.r_obstacle[self.route]
        half = 0.5 * sizes[None, :, None]
        d_box = ms.point_rect_distance(xy[:, None], torch.cat((bxy - half, bxy + half), -1))[:, 0]
        d = torch.stack((d_static, d_box, self.mover_gaps(xy), self.robot_gaps(xy)), -1)
        return d.argmin(-1)

    def _sample_movers(self, env_ids, r, chunk: int = 256) -> None:
        """env_ids 의 mover 구간을 새로 뽑는다 (벽, 선반, 박스 회피)."""
        sizes = torch.tensor(self.routes.obstacle_sizes, device=self.device)
        bounds = tuple(float(v) for v in self.routes_bounds)
        for c0 in range(0, len(env_ids), chunk):
            ids, rr = env_ids[c0 : c0 + chunk], r[c0 : c0 + chunk]
            xy = self.r_obstacle[rr]  # (n, slots, 2)
            half = 0.5 * sizes[None, :, None]
            boxes = torch.cat((xy - half, xy + half), -1)
            rects = torch.cat((self.static_rects.expand(len(ids), -1, -1), boxes), 1)
            for shape, legs in zip(ms.MOVERS, self.mover_legs, strict=True):
                new = ms.sample_legs(
                    rects,
                    bounds,
                    self.r_path[rr],
                    self.r_npts[rr],
                    self.r_start[rr],
                    self.r_goal[rr],
                    shape,
                    self.arena,
                )
                for k, v in new.items():
                    legs[k][ids] = v

    def mover_state(self, ids=None):
        """mover 별 위치, yaw, 속도, 활성 (env 프레임)."""
        ids = slice(None) if ids is None else ids
        out = []
        for i, legs in enumerate(self.mover_legs):
            sub = {k: v[ids] for k, v in legs.items()}
            pos, yaw, vel = ms.mover_pose(sub, self.mover_t[i, ids])
            act = sub["active"]
            pos = torch.where(act[:, None], pos, self.mover_park[i].expand_as(pos))
            moving = act & ~self.mover_held[i, ids]
            vel = torch.where(moving[:, None], vel, torch.zeros_like(vel))
            out.append((pos, yaw, vel, act))
        return out

    def _write_movers(self, ids) -> None:
        origins = self.scene.env_origins[ids]
        for obj, (pos, yaw, _, _) in zip(self.mover_objs, self.mover_state(ids), strict=True):
            pose = obj.data.default_root_pose.torch[ids].clone()
            pose[:, :2] = origins[:, :2] + pos
            z = torch.zeros_like(yaw)
            pose[:, 3:7] = torch.stack((z, z, torch.sin(yaw / 2), torch.cos(yaw / 2)), -1)
            # 정지 상태로 스텝마다 순간이동: 라이다가 mover_state() 를 그대로 봄
            v6 = torch.zeros(len(pos), 6, device=self.device)
            obj.write_root_pose_to_sim_index(root_pose=pose, env_ids=ids)
            obj.write_root_velocity_to_sim_index(root_velocity=v6, env_ids=ids)

    def mover_gaps(self, xy: torch.Tensor, ids=None) -> torch.Tensor:
        """(n,) 로봇 중심에서 활성 mover 표면까지 거리, 사람 안이면 음수, 없으면 inf."""
        gap = torch.full((len(xy),), float("inf"), device=self.device)
        for shape, (pos, yaw, _, act) in zip(ms.MOVERS, self.mover_state(ids), strict=True):
            g = ms.surface_gap(shape, xy, pos, yaw)
            gap = torch.where(act, torch.minimum(gap, g), gap)
        return gap

    def action_target(self, actions: torch.Tensor) -> torch.Tensor:
        if not (self._routes_ready and self.cap_active):
            return super().action_target(actions)  # ppo_k 변환 그대로
        return ts.action_to_target(actions, v_cap=self.v_cap)

    def reward_v_cap(self) -> torch.Tensor | None:
        return self.v_cap if self._routes_ready and self.cap_active else None

    def wait_scale(self):
        """이동 시간 대기는 에피소드 상한을 따름: 고정 상한이면 1, 아니면 (n,)."""
        return ts.NAV_SPEED_MPS / self.v_cap if self.cap_active else 1.0

    def _pre_physics_step(self, actions: torch.Tensor) -> None:
        if self._routes_ready:
            self.prev_target = self.target.clone()
        super()._pre_physics_step(actions)
        if self._routes_ready:
            s = self.spec
            self.target_flipped, self.target_sign = ts.flip_step(
                self.target[:, 1], self.target_sign, s.flip_deadband
            )
            flipped, self.cmd_sign = ts.flip_step(self.cmd[:, 1], self.cmd_sign, s.flip_deadband)
            self.cmd_flips += flipped.float()
            xy, _ = self.robot_pose()
            for i, (shape, legs) in enumerate(zip(ms.MOVERS, self.mover_legs, strict=True)):
                t, self.mover_wait[i], _ = ms.yield_step(
                    legs,
                    self.mover_t[i],
                    self.step_dt,
                    shape,
                    xy,
                    self.mover_yield[i],
                    self.mover_wait[i],
                    self.arena.mover_wait_s * self.wait_scale(),
                )
                self.mover_held[i] = t == self.mover_t[i]
                self.mover_t[i] = t
            all_ids = torch.arange(self.num_envs, device=self.device)
            self._write_movers(all_ids)
            a = self.arena
            for k, route in enumerate(self.robot_routes):
                # 앞선 로봇이 먼저 움직인 뒤: 한 스텝에 두 로봇이 한 노드를 못 잡음
                blocked = self.blocked_nodes(k) if a.robot_reserve else None
                yield_m = torch.where(self.robot_priority[k], 0.0, a.robot_yield_m)
                self.robot_t[k], self.robot_wait[k], self.robot_back[k] = rbs.robot_step(
                    route,
                    self.robot_t[k],
                    self.robot_wait[k],
                    self.robot_back[k],
                    self.step_dt,
                    xy,
                    yield_m,
                    a.robot_wait_s * self.wait_scale(),
                    a.robot_backoff_s * self.wait_scale(),
                    retreat=bool(a.robot_retreat),
                    lock_m=a.robot_lock_m,
                    blocked=blocked,
                    node_m=a.robot_reserve_m,
                )
            if self.robot_objs:
                self._write_robots(all_ids)

    def obstacle_xy(self) -> torch.Tensor:
        """(N, slots, 2) 장애물 박스 중심 (env 프레임)."""
        org = self.scene.env_origins[:, :2]
        return torch.stack([o.data.root_pos_w.torch[:, :2] - org for o in self.obstacles], 1)

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        xy, yaw = self.robot_pose()
        self._follow(torch.arange(self.num_envs, device=self.device), xy, yaw)
        self.mover_gap = torch.minimum(self.mover_gap, self.mover_gaps(xy))
        self.robot_gap_min = torch.minimum(self.robot_gap_min, self.robot_gaps(xy))
        self.hit_kind = self.nearest_hit_kind(xy)
        self.success, self.collided, self.oob = ts.episode_outcome(
            self.d_goal, self.max_contact_force(), xy, self.start_xy, self.spec
        )
        clear = self.footprint_clearance(xy, yaw)
        graze = arena.graze_flags(clear, self.arena.graze_m)
        self.graze_only = graze & ~self.collided  # 접촉 센서가 못 본 것
        self.collided = self.collided | graze
        self.success = self.success & ~graze
        self.oob = self.oob & ~graze
        return ts.split_dones(
            self.success | self.collided | self.oob,
            self.episode_length_buf,
            self.ep_limit if self.cap_active else self.max_episode_length,
        )

    def footprint_clearance(self, xy, yaw) -> torch.Tensor:
        """(n,) 우리 발자국에서 가장 가까운 장애물까지 거리, 벽/선반/박스 몫은 self.wall_clear 에 저장."""
        n = len(xy)
        xy, h = arena.footprint_box(xy, yaw, self.arena.footprint_x, self.arena.footprint_y)
        sizes = torch.tensor(self.routes.obstacle_sizes, device=self.device)
        bxy = self.r_obstacle[self.route]
        half = 0.5 * sizes[None, :, None]
        boxes = torch.cat((bxy - half, bxy + half), -1)
        rects = torch.cat((self.static_rects.expand(n, -1, -1), boxes), 1)
        self.wall_clear = arena.footprint_clearance(xy, yaw, h, rects)
        clear = self.wall_clear
        inf = torch.full_like(clear, float("inf"))
        for shape, (pos, myaw, _, act) in zip(ms.MOVERS, self.mover_state(), strict=True):
            if shape.kind == "cylinder":
                c = arena.cylinder_clearance(xy, yaw, h, pos, shape.size[0])
            else:
                hm = (0.5 * shape.size[0], 0.5 * shape.size[1])
                c = arena.box_clearance(xy, yaw, h, pos, myaw, hm)
            clear = torch.minimum(clear, torch.where(act, c, inf))
        hr = (0.5 * rbs.ROBOT.size[0], 0.5 * rbs.ROBOT.size[1])
        for pos, ryaw, act in self.robot_state():
            c = arena.box_clearance(xy, yaw, h, pos, ryaw, hr)
            clear = torch.minimum(clear, torch.where(act, c, inf))
        return clear

    def _get_rewards(self) -> torch.Tensor:
        rew = super()._get_rewards()
        if not self._routes_ready:
            return rew
        a = self.arena
        pen = arena.near_wall_penalty(self.wall_clear, a.near_wall_m, a.w_near_wall)
        terms = {"near_wall": pen}
        s = self.spec
        if s.w_action_rate or s.w_w_flip:
            terms.update(
                ts.action_rate_terms(self.target, self.prev_target, self.target_flipped, s)
            )
        for k, v in terms.items():
            self.ep_reward_terms.setdefault(k, torch.zeros_like(v))
            self.ep_reward_terms[k] += v
        return rew + sum(terms.values())

    def lidar_history(self, ranges: torch.Tensor) -> torch.Tensor | None:
        """이전 섹터 벡터 (n, frames - 1, sectors), 최신 먼저, stride 스텝 간격.
        새 에피소드는 모든 칸을 현재 프레임으로 채움. 스텝마다 한 번 부름."""
        s = self.spec
        if s.lidar_frames <= 1:
            return None
        cur = ts.lidar_sectors(ranges, s) / s.lidar_obs_max
        buf = torch.cat((cur[:, None], self.lid_buf[:, :-1]), 1)
        fresh = self.lid_fresh[:, None, None]
        self.lid_buf = torch.where(fresh, cur[:, None].expand_as(buf), buf)
        self.lid_fresh[:] = False
        return self.lid_buf[:, s.lidar_stride_steps :: s.lidar_stride_steps]

    def privileged_states(self, xy, yaw) -> torch.Tensor:
        """크리틱 전용 항목: 움직이는 물체와 로봇의 실제 위치, 속도."""
        pos, vel, on = [], [], []
        for p, _, v, a_ in self.mover_state():
            pos.append(p)
            vel.append(v)
            on.append(a_)
        for k, (p, _, a_) in enumerate(self.robot_state()):
            pos.append(p)
            vel.append((p - self.robot_prev_xy[:, k]) / self.step_dt)
            on.append(a_)
            self.robot_prev_xy[:, k] = p
        return ts.privileged_block(
            torch.stack(pos, 1), torch.stack(vel, 1), torch.stack(on, 1), xy, yaw
        )

    def _get_observations(self) -> dict:
        if not self._routes_ready:
            out = super()._get_observations()  # 경로가 생기기 전엔 intent 없음
            if self.cfg.state_space:
                out["critic"] = torch.zeros(self.num_envs, self.cfg.state_space, device=self.device)
            return out
        intent = mask = prio = None
        xy, yaw = self.robot_pose()
        if self.spec.intent_robots:
            intent, mask, prio = self.robot_intent(xy, yaw)
        ranges = self.lidar_ranges()
        obs = ts.build_observation(
            ranges,
            self.la_dist,
            self.la_ang,
            self.body_velocity(),
            self.spec,
            intent=intent,
            intent_mask=mask,
            intent_priority=prio,
            prev_target=self.target,
            lidar_history=self.lidar_history(ranges),
            speed_cap=self.v_cap,
        )
        out = {"policy": obs}
        if self.cfg.state_space:
            out["critic"] = self.privileged_states(xy, yaw)
        return out
