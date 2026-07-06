"""app layer — headless orchestration (Claude.md §14).

robot_test_controller: manual jog/home logic behind the Robot Test tab.
cycle_manager: the automatic detect -> plan -> PLC pick/place cycle.
"""

from .cycle_manager import (
    CycleConfig,
    CycleManager,
    CycleResult,
    CycleStep,
    DryRunJobRunner,
    HandshakeJobRunner,
    JobRunner,
    ScriptedTargetSource,
    TargetSource,
    VisionTargetSource,
    default_scripted_targets,
    make_job_runner,
)
from .recipes import Recipe, RecipeError, RecipeStore, slugify_key
from .robot_test_controller import (
    RobotTestController,
    build_dry_run_controller,
)

__all__ = [
    "CycleConfig",
    "CycleManager",
    "CycleResult",
    "CycleStep",
    "DryRunJobRunner",
    "HandshakeJobRunner",
    "JobRunner",
    "Recipe",
    "RecipeError",
    "RecipeStore",
    "RobotTestController",
    "ScriptedTargetSource",
    "TargetSource",
    "VisionTargetSource",
    "build_dry_run_controller",
    "default_scripted_targets",
    "make_job_runner",
    "slugify_key",
]
