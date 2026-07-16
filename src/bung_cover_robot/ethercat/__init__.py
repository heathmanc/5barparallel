"""EtherCAT motion layer — the PC as CiA 402 master over StepperOnline A6 drives.

The PC is the motion controller: it plans validated Cartesian trajectories and
streams Cyclic-Synchronous-Position setpoints to the two A6 servo drives.

  * cia402      — the CiA 402 drive state machine (pure logic).
  * trajectory  — Cartesian straight-line planner -> per-cycle joint setpoints
                  (pure logic; every point workspace-validated at plan time).

The pysoem master + real-time streamer and the RobotDriver implementation are
added in later stages, behind these tested cores.
"""

from .cia402 import (
    Cia402State,
    MODE_CSP,
    MODE_HOMING,
    decode_state,
    is_fault,
    is_operation_enabled,
    next_controlword,
)
from .trajectory import (
    JointSetpoint,
    Trajectory,
    TrajectoryError,
    TrajectoryLimits,
    plan_linear_move,
)

__all__ = [
    "Cia402State",
    "MODE_CSP",
    "MODE_HOMING",
    "decode_state",
    "is_fault",
    "is_operation_enabled",
    "next_controlword",
    "JointSetpoint",
    "Trajectory",
    "TrajectoryError",
    "TrajectoryLimits",
    "plan_linear_move",
]
