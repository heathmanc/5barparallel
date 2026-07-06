"""Automatic pick/place cycle (Claude.md §12-§15) — closing the loop.

Ties the whole stack together for a full battery::

    capture -> detect holes (drop targets) + covers (pick candidates)
    -> pixel->robot via calibration -> WorkspaceValidator guard
    -> plan pick & drop angles (robot.planner) -> PLC pick/place handshake
    -> re-image (loose covers shift each pick) -> next hole

Holes and covers share one calibration plane (Claude.md §13). A battery-type
changeover swaps the active homography (per-recipe), so the cycle only ever needs
one ``calibration`` — the same pixel->robot transform maps both the vent holes and
the loose covers.

The manager owns no Qt; the Vision tab is a thin view that calls ``run_cycle`` and
renders the returned steps. Every target is validated before a job is built, so a
bad pose can never reach the PLC.

Two job runners let the same cycle run with or without a PLC:
  * ``HandshakeJobRunner`` — the real §11 handshake over a PlcClient (also the
    SimulatedPlcClient, so ``--sim-plc`` and tests exercise the full protocol).
  * ``DryRunJobRunner`` — no PLC: drives the pick then drop angles through the
    dry-run driver and always succeeds, so the orchestration/validation logic
    runs end-to-end with nothing connected.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

from ..plc.handshake import JobResult, PickPlaceHandshake
from ..robot.driver import RobotDriver
from ..robot.planner import PickPlaceJob, PlanningError, make_job, sort_holes_along_conveyor
from ..vision.calibration import HomographyTransform
from ..vision.camera import Camera, CameraError
from ..vision.detect_covers import CoverDetector
from ..vision.detect_holes import HoleDetector
from .robot_test_controller import RobotTestController

Point = Tuple[float, float]


# --------------------------------------------------------------------------- #
# Job runners
# --------------------------------------------------------------------------- #
class JobRunner(ABC):
    @abstractmethod
    def run(self, job: PickPlaceJob) -> JobResult:
        ...


class HandshakeJobRunner(JobRunner):
    """Run each job through the real §11 pick/place handshake over a PlcClient."""

    def __init__(self, client, command_timeout_s: float = 30.0) -> None:
        self.handshake = PickPlaceHandshake(client, command_timeout_s=command_timeout_s)

    def run(self, job: PickPlaceJob) -> JobResult:
        return self.handshake.send_job_and_wait(
            (job.pick.left_deg, job.pick.right_deg),
            (job.drop.left_deg, job.drop.right_deg),
            job.hole_index,
            job.cover_id,
        )


class DryRunJobRunner(JobRunner):
    """No PLC: move the dry-run driver through pick then drop, always succeed."""

    def __init__(self, driver: RobotDriver) -> None:
        self.driver = driver
        self._id = 0

    def run(self, job: PickPlaceJob) -> JobResult:
        self._id += 1
        self.driver.move_to_angles(job.pick.left_deg, job.pick.right_deg)
        self.driver.move_to_angles(job.drop.left_deg, job.drop.right_deg)
        return JobResult(True, "ok (dry-run)", self._id)


def make_job_runner(driver: RobotDriver, command_timeout_s: float = 30.0) -> JobRunner:
    """A PlcClient-backed driver gets the real handshake; a dry-run driver the
    simulated runner. Detected via the driver's optional ``client`` attribute."""
    client = getattr(driver, "client", None)
    if client is not None:
        return HandshakeJobRunner(client, command_timeout_s=command_timeout_s)
    return DryRunJobRunner(driver)


# --------------------------------------------------------------------------- #
# Results
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class CycleStep:
    hole_index: int
    drop_xy: Point
    cover_id: int
    pick_xy: Optional[Point]
    ok: bool
    reason: str


@dataclass
class CycleResult:
    steps: List[CycleStep] = field(default_factory=list)
    ok: bool = False
    reason: str = ""

    @property
    def placed(self) -> List[CycleStep]:
        return [s for s in self.steps if s.ok]


@dataclass
class CycleConfig:
    max_holes: int = 6
    used_tolerance_mm: float = 12.0  # a picked cover shouldn't be re-picked


# --------------------------------------------------------------------------- #
# Target sources — where hole/cover targets come from (vision, or a bypass)
# --------------------------------------------------------------------------- #
class TargetSource(ABC):
    """Supplies robot-frame hole (drop) and cover (pick) targets to the cycle.

    Swapping the source is how vision is bypassed for testing: the real vision
    path detects+calibrates; the scripted path returns fixed coordinates so the
    plan -> validate -> PLC pick/place handshake loop can be exercised with no
    camera, detection, or calibration at all.
    """

    label = "targets"

    def preflight(self) -> Optional[str]:
        """Source-specific blocking reason (e.g. missing calibration), or None."""
        return None

    @abstractmethod
    def holes(self) -> List[Point]:
        """Robot-frame hole (drop) targets. Called once at cycle start."""

    @abstractmethod
    def covers(self) -> List[Point]:
        """Robot-frame candidate cover (pick) targets. Called before each pick
        (a real camera re-images; scripted returns its fixed list)."""


class VisionTargetSource(TargetSource):
    """The real path: detect holes + covers, map pixel->robot via calibration."""

    label = "vision"

    def __init__(self, camera, calibration, validator, hole_detector, cover_detector):
        self.camera = camera
        self.calibration = calibration
        self.validator = validator
        self.hole_detector = hole_detector
        self.cover_detector = cover_detector

    def preflight(self) -> Optional[str]:
        if self.calibration is None:
            return "no pixel->robot calibration — build one in the Calibration tab"
        return None

    def holes(self) -> List[Point]:
        res = self.hole_detector.detect(self.camera.grab())
        return [self.calibration.pixel_to_robot(c.cx, c.cy) for c in res.holes]

    def covers(self) -> List[Point]:
        res = self.cover_detector.detect(
            self.camera.grab(), self.calibration.pixel_to_robot, self.validator
        )
        return [c.robot_xy for c in res.accepted if c.robot_xy is not None]


class ScriptedTargetSource(TargetSource):
    """Vision bypass: fixed robot-frame targets, no camera/detection/calibration.

    Covers are returned in full each call; the cycle's used-position filter
    consumes them one per successful pick, so it terminates like the real thing.
    """

    label = "scripted (vision bypass)"

    def __init__(self, holes_xy, covers_xy):
        self._holes = [(float(x), float(y)) for x, y in holes_xy]
        self._covers = [(float(x), float(y)) for x, y in covers_xy]

    def holes(self) -> List[Point]:
        return list(self._holes)

    def covers(self) -> List[Point]:
        return list(self._covers)


def default_scripted_targets(controller, hole_count: int = 6) -> "tuple":
    """A deterministic, reachable set of scripted holes + covers for bypass tests,
    derived from the robot's own workspace (so every target is valid)."""
    val = controller.validator
    home_y = controller.home_xy[1]

    def row(y: float, n: int, span: float = 140.0) -> List[Point]:
        if n <= 1:
            xs = [0.0]
        else:
            xs = [-span + 2 * span * i / (n - 1) for i in range(n)]
        return [(round(x, 1), y) for x in xs if val.validate(x, y).ok]

    holes = row(home_y, hole_count)
    covers = row(home_y - 45.0, max(hole_count, 6))
    if len(covers) < len(holes):                 # fall back to the hole row's Y
        covers = row(home_y, max(hole_count, 6))
    return holes, covers


# --------------------------------------------------------------------------- #
# Cycle manager
# --------------------------------------------------------------------------- #
class CycleManager:
    def __init__(
        self,
        controller: RobotTestController,
        camera: Camera,
        calibration: Optional[HomographyTransform],
        hole_detector: Optional[HoleDetector] = None,
        cover_detector: Optional[CoverDetector] = None,
        job_runner: Optional[JobRunner] = None,
        config: Optional[CycleConfig] = None,
        target_source: Optional[TargetSource] = None,
    ) -> None:
        self.controller = controller
        self.camera = camera
        # One plane for both holes and covers; changeover swaps this per battery.
        self.calibration = calibration
        self.hole_detector = hole_detector or HoleDetector()
        self.cover_detector = cover_detector or CoverDetector()
        self.job_runner = job_runner
        self.config = config or CycleConfig()
        # Where targets come from. Default = the real vision path; pass a
        # ScriptedTargetSource to bypass vision for testing.
        self.target_source = target_source or VisionTargetSource(
            camera, calibration, controller.validator,
            self.hole_detector, self.cover_detector,
        )

    # --- preflight ----------------------------------------------------------
    def _motion_block(self) -> Optional[str]:
        if not self.controller.is_enabled:
            return "drives are disabled — enable them in Robot Test first"
        if not self.controller.is_referenced:
            return "robot is not referenced — Home (find ref) in Robot Test first"
        return None

    def preflight(self) -> Optional[str]:
        """Return a blocking reason, or None if the cycle may run."""
        return self._motion_block() or self.target_source.preflight()

    # --- run ----------------------------------------------------------------
    def run_cycle(
        self,
        should_stop: Optional[Callable[[], bool]] = None,
        on_step: Optional[Callable[[CycleStep], None]] = None,
    ) -> CycleResult:
        block = self.preflight()
        if block is not None:
            return CycleResult(ok=False, reason=block)

        runner = self.job_runner or make_job_runner(self.controller.driver)
        kin, validator = self.controller.kin, self.controller.validator

        # Holes -> ordered drop targets (robot frame), from vision or a bypass.
        try:
            hole_xy = self.target_source.holes()
        except CameraError as exc:
            return CycleResult(ok=False, reason=f"capture failed: {exc}")
        if not hole_xy:
            return CycleResult(ok=False, reason="no holes detected")
        order = sort_holes_along_conveyor(hole_xy)[: self.config.max_holes]

        result = CycleResult()
        used: List[Point] = []
        for cover_id, hi in enumerate(order):
            if should_stop is not None and should_stop():
                result.reason = "stopped by operator"
                break

            drop_xy = hole_xy[hi]
            cover = self._pick_candidate(drop_xy, used)
            if cover is None:
                step = CycleStep(hi, drop_xy, -1, None, False, "no reachable cover available")
                result.steps.append(step)
                if on_step is not None:
                    on_step(step)
                result.reason = "out of reachable covers"
                break  # supply exhausted — stop cleanly

            try:
                job = make_job(
                    kin, validator, hole_index=hi, cover_id=cover_id,
                    pick_xy=cover, drop_xy=drop_xy,
                )
            except PlanningError as exc:
                step = CycleStep(hi, drop_xy, cover_id, cover, False, str(exc))
                result.steps.append(step)
                if on_step is not None:
                    on_step(step)
                continue  # this hole/cover pairing is unreachable — try next hole

            job_res = runner.run(job)
            step = CycleStep(hi, drop_xy, cover_id, cover, job_res.ok, job_res.reason)
            result.steps.append(step)
            if on_step is not None:
                on_step(step)
            if job_res.ok:
                used.append(cover)
            else:
                result.reason = f"job failed: {job_res.reason}"
                result.ok = False
                return result  # a PLC fault/timeout stops the cycle

        placed = len(result.placed)
        if not result.reason:
            result.reason = f"placed {placed}/{len(order)} covers"
        result.ok = placed > 0
        return result

    # --- single job (vision + selection bypass) -----------------------------
    def run_single_job(self, pick_xy: Point, drop_xy: Point) -> CycleStep:
        """Fire ONE pick->place job through the PLC handshake, from explicit
        robot-frame points. No vision, no cover selection — the simplest way to
        exercise IK + validation + the §11 handshake on hardware."""
        block = self._motion_block()
        if block is not None:
            return CycleStep(0, drop_xy, 0, pick_xy, False, block)
        runner = self.job_runner or make_job_runner(self.controller.driver)
        try:
            job = make_job(self.controller.kin, self.controller.validator,
                           hole_index=0, cover_id=0, pick_xy=pick_xy, drop_xy=drop_xy)
        except PlanningError as exc:
            return CycleStep(0, drop_xy, 0, pick_xy, False, str(exc))
        res = runner.run(job)
        return CycleStep(0, drop_xy, 0, pick_xy, res.ok, res.reason)

    # --- cover selection ----------------------------------------------------
    def _pick_candidate(self, drop_xy: Point, used: List[Point]) -> Optional[Point]:
        """Re-source and return the reachable cover nearest the target hole that
        hasn't already been picked, or None if none remain."""
        try:
            candidates = self.target_source.covers()
        except CameraError:
            return None
        val = self.controller.validator
        reachable = [
            xy for xy in candidates
            if xy is not None and val.validate(*xy).ok and not self._is_used(xy, used)
        ]
        if not reachable:
            return None
        return min(reachable, key=lambda xy: _dist2(xy, drop_xy))

    def _is_used(self, xy: Point, used: List[Point]) -> bool:
        tol2 = self.config.used_tolerance_mm ** 2
        return any(_dist2(xy, u) <= tol2 for u in used)


def _dist2(a: Point, b: Point) -> float:
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
