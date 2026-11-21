"""Shared Pinky (Pode) articulation and Newton physics config for the sim checks.

Import only after ``launch_simulation`` has started (Isaac Lab modules load lazily).
"""

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg

PINKY_USD = os.environ.get("PINKY_USD", r"C:\isaac_assets\pinky\usd\nomerge\pinky\pinky.usda")

WHEEL_RADIUS = 0.028  # URDF collision sphere radius [m]
WHEEL_SEPARATION = 0.0961  # vendor Gazebo diff-drive plugin value [m]
BODY = "Geometry/base_footprint/base_link"  # articulation root body under the robot prim


def make_newton_cfg(njmax: int = 120, nconmax: int = 60) -> NewtonCfg:
    """MuJoCo-Warp settings modelled on the Ant task (implicitfast, pyramidal cone)."""
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
    """Pinky from the converted USD: wheels in velocity mode (kp=0, kd>0), casters passive."""
    return ArticulationCfg(
        prim_path=prim_path,
        spawn=sim_utils.UsdFileCfg(usd_path=PINKY_USD, activate_contact_sensors=True),
        # base_footprint (the spawned prim's child) sits at the robot prim origin;
        # base_link is 0.028 above it.
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
