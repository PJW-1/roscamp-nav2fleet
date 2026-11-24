"""시뮬 점검용 공용 Pinky(Pode) 관절체와 Newton 물리 설정.

launch_simulation 이 시작된 뒤에만 import 한다 (Isaac Lab 모듈은 지연 로드).
"""

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg

PINKY_USD = os.environ.get("PINKY_USD", r"C:\isaac_assets\pinky\usd\nomerge\pinky\pinky.usda")

WHEEL_RADIUS = 0.028  # URDF 충돌 구 반지름 [m]
WHEEL_SEPARATION = 0.0961  # 제조사 Gazebo diff-drive 값 [m]
BODY = "Geometry/base_footprint/base_link"  # 로봇 prim 아래 관절체 루트 몸체


def make_newton_cfg(njmax: int = 120, nconmax: int = 60) -> NewtonCfg:
    """Ant 과제 식 MuJoCo-Warp 설정 (implicitfast, pyramidal cone)."""
    return NewtonCfg(
        solver_cfg=MJWarpSolverCfg(
            njmax=njmax,
            nconmax=nconmax,
            cone="pyramidal",
            integrator="implicitfast",
            impratio=1,
        ),
        num_substeps=1,
        debug_mode=False,
    )


def make_pinky_cfg(
    prim_path: str = "{ENV_REGEX_NS}/Robot",
    wheel_damping: float = 0.05,
    wheel_effort: float = 0.5,
    caster_damping: float = 0.0,
) -> ArticulationCfg:
    """변환된 USD 의 Pinky: 바퀴는 속도 모드(kp=0, kd>0), 캐스터는 수동."""
    return ArticulationCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(usd_path=PINKY_USD, activate_contact_sensors=True),
        # base_footprint 는 로봇 prim 원점, base_link 는 그 위 0.028
        init_state=ArticulationCfg.InitialStateCfg(pos=(0.0, 0.0, 0.002), joint_pos={".*": 0.0}),
        actuators={
            "wheels": ImplicitActuatorCfg(
                joint_names_expr=["l_wheel_joint", "r_wheel_joint"],
                stiffness=0.0,
                damping=wheel_damping,
                effort_limit_sim=wheel_effort,
                velocity_limit_sim=30.0,
            ),
            "casters": ImplicitActuatorCfg(
                joint_names_expr=["caster_rotate_joint", "caster_wheel_joint"],
                stiffness=0.0,
                damping=caster_damping,
            ),
        },
    )
