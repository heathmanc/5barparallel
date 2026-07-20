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
bad pose can never reach the drives.

The PC is the motion controller (EtherCAT): a job is just a pick move followed by
a drop move, driven straight through the ``RobotDriver`` — ``DirectJobRunner``
works for the dry-run driver and the real EtherCAT driver alike (the latter plans
a coordinated straight-line move inside ``move_to_angles``).
"""

from __future__ import annotations

import logging
import math
import random
import time
from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Deque, List, Optional, Tuple

from ..robot.driver import RobotDriver, RobotDriverError
from ..robot.planner import PickPlaceJob, PlanningError, make_job, sort_holes_along_conveyor
from ..vision.calibration import HomographyTransform
from ..vision.camera import Camera, CameraError
from ..vision.detect_covers import CoverDetector
from ..vision.detect_holes import HoleDetector
from .robot_test_controller import RobotTestController

logger = logging.getLogger(__name__)

Point = Tuple[float, float]


# --------------------------------------------------------------------------- #
# Job runners
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class JobResult:
    """Outcome of one pick/place job."""

    ok: bool
    reason: str
    command_id: int = 0


class JobRunner(ABC):
    @abstractmethod
    def run(self, job: PickPlaceJob) -> JobResult:
        ...


@dataclass(frozen=True)
class PickSequence:
    """Timing for the air-cylinder + vacuum pick head.

    The dwells cover the real pneumatics: the cylinder needs a moment to finish
    extending/retracting, and the vacuum needs a moment to build (grip) or bleed
    (release). Zero them in tests to keep the sim fast."""

    plunge_dwell_s: float = 0.15    # let the cylinder finish extending/retracting
    grip_dwell_s: float = 0.20      # let vacuum build before lifting
    release_dwell_s: float = 0.20   # let the part drop before lifting


class DirectJobRunner(JobRunner):
    """Drive the RobotDriver through a full pick-and-place, actuating the pick
    head's air cylinder and vacuum along the way.

    The PC owns motion, so each move is one coordinated ``move_to_angles``. The
    sequence is: travel to the cover, plunge the cylinder onto it, pull vacuum,
    lift, travel to the hole, plunge, vent the vacuum to release, lift. Works for
    the dry-run driver (instant, I/O no-ops) and the EtherCAT driver (streamed
    moves, DO-driven tooling) alike.

    On any error the head is left safe — vacuum vented and cylinder retracted —
    so a fault mid-move can't strand a cover on the cup or the plunger down."""

    def __init__(self, driver: RobotDriver,
                 sequence: Optional[PickSequence] = None,
                 sleep: Callable[[float], None] = time.sleep,
                 move_speed_mm_s: Optional[float] = None) -> None:
        self.driver = driver
        self.sequence = sequence or PickSequence()
        self._sleep = sleep
        # Optional Cartesian speed cap for the travel moves (None = the driver's
        # configured limit). The bench demo passes a gentle speed so a big move
        # can't outrun the servo and trip an excessive-position-deviation alarm.
        self.move_speed_mm_s = move_speed_mm_s
        self._id = 0

    def run(self, job: PickPlaceJob) -> JobResult:
        self._id += 1
        try:
            # Pick: travel over the cover, plunge, grip, lift.
            self.driver.move_to_angles(job.pick.left_deg, job.pick.right_deg,
                                       speed_mm_s=self.move_speed_mm_s)
            self._grip()
            # Place: travel over the hole, plunge, release, lift.
            self.driver.move_to_angles(job.drop.left_deg, job.drop.right_deg,
                                       speed_mm_s=self.move_speed_mm_s)
            self._release()
        except RobotDriverError as exc:
            self._make_safe()
            return JobResult(False, str(exc), self._id)
        return JobResult(True, "ok", self._id)

    def _grip(self) -> None:
        seq = self.sequence
        self.driver.set_plunger(True)
        self._sleep(seq.plunge_dwell_s)
        self.driver.set_vacuum(True)
        self._sleep(seq.grip_dwell_s)
        self.driver.set_plunger(False)

    def _release(self) -> None:
        seq = self.sequence
        self.driver.set_plunger(True)
        self._sleep(seq.plunge_dwell_s)
        self.driver.set_vacuum(False)
        self._sleep(seq.release_dwell_s)
        self.driver.set_plunger(False)

    def _make_safe(self) -> None:
        """Best-effort: vent vacuum and retract the cylinder after a failure."""
        try:
            self.driver.set_vacuum(False)
            self.driver.set_plunger(False)
        except RobotDriverError:
            pass


def make_job_runner(driver: RobotDriver,
                    sequence: Optional[PickSequence] = None,
                    move_speed_mm_s: Optional[float] = None) -> JobRunner:
    """Every driver runs jobs the same way: travel to the cover, grip, travel to
    the hole, release — actuating the pick head at each end. ``move_speed_mm_s``
    optionally caps the travel speed (the bench demo runs gentle)."""
    return DirectJobRunner(driver, sequence, move_speed_mm_s=move_speed_mm_s)


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
    plan -> validate -> move loop can be exercised with no camera, detection, or
    calibration at all.
    """

    label = "targets"
    last_frame = None          # most recent camera frame imaged (None if source-less)

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
        self.last_frame = None          # exposed so the GUI can live-render each grab

    def preflight(self) -> Optional[str]:
        if self.calibration is None:
            return "no pixel->robot calibration — build one in the Calibration tab"
        return None

    def holes(self) -> List[Point]:
        frame = self.camera.grab()
        self.last_frame = frame
        res = self.hole_detector.detect(frame)
        return [self.calibration.pixel_to_robot(c.cx, c.cy) for c in res.holes]

    def covers(self) -> List[Point]:
        frame = self.camera.grab()
        self.last_frame = frame
        res = self.cover_detector.detect(
            frame, self.calibration.pixel_to_robot, self.validator
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
# Bench demo — fixed pick nest + a variably-placed cover row (vision bypass)
# --------------------------------------------------------------------------- #
def demo_pick_and_place_targets(
    validator,
    home_xy: Point,
    holes: int = 6,
    spacing_mm: float = 35.0,
    rng: Optional[random.Random] = None,
) -> Tuple[Point, List[Point]]:
    """A fixed supply nest + a variably-placed straight-line battery of ``holes``
    bung-cover holes, ``spacing_mm`` apart, every point workspace-validated.

    Stands in for vision while calibration is pending: the robot always picks
    from the *same* nest and drops into a row of covers laid out exactly like a
    real battery — six holes, 35 mm pitch, in a line — that sits at a different
    position and tilt each call. The pitch is held fixed (that's the battery);
    only the row's placement is randomised. If a random pose can't fit the whole
    row in reach we fall back to a centred, axis-aligned row so the demo always
    returns something drivable."""
    rng = rng or random.Random()
    hx, hy = float(home_xy[0]), float(home_xy[1])

    # Fixed supply nest: the first reachable candidate off to the near side.
    nest = _first_valid(validator, [
        (hx + 110.0, hy - 40.0), (hx + 90.0, hy - 20.0),
        (hx - 90.0, hy - 20.0), (hx, hy - 60.0), (hx, hy),
    ]) or (hx, hy)

    def row_at(cx: float, cy: float, theta: float) -> List[Point]:
        pts = []
        for i in range(holes):
            off = (i - (holes - 1) / 2.0) * spacing_mm
            pts.append((round(cx + off * math.cos(theta), 1),
                        round(cy + off * math.sin(theta), 1)))
        return pts

    def all_valid(pts: List[Point]) -> bool:
        return all(validator.validate(x, y).ok for x, y in pts)

    # Random placements first — a spread of centres and tilts to exercise motion.
    for _ in range(80):
        cx = hx + rng.uniform(-35.0, 35.0)
        cy = hy + rng.uniform(-25.0, 25.0)
        theta = rng.uniform(-0.30, 0.30)          # +/- ~17 deg tilt
        pts = row_at(cx, cy, theta)
        if all_valid(pts):
            return nest, pts

    # Fallback: a centred, axis-aligned row at the same (fixed) pitch.
    centred = row_at(hx, hy, 0.0)
    if all_valid(centred):
        return nest, centred
    # Last resort: whatever holes of the centred row validate.
    return nest, [p for p in centred if validator.validate(*p).ok]


def _first_valid(validator, candidates: List[Point]) -> Optional[Point]:
    for x, y in candidates:
        if validator.validate(x, y).ok:
            return (round(x, 1), round(y, 1))
    return None


DEMO_MOVE_SPEED_MM_S = 60.0   # gentle default so the demo can't outrun the servo
REENABLE_ATTEMPTS = 4         # retries for a transient enable-chain blip
REENABLE_DELAY_S = 0.05       # first backoff wait; doubles each try (capped 0.5 s)


def _run_with_reenable(driver, run_job, *, should_stop=None,
                       attempts=REENABLE_ATTEMPTS, base_delay=REENABLE_DELAY_S):
    """Run ``run_job`` and, if it fails because the drive dropped out of Operation
    Enabled WITHOUT a fault, re-enable and retry with exponential backoff.

    The AS715N has no STO — its power stage is fed through the E-stop contactor,
    so a marginal connection can momentarily drop bus power: the drive falls to
    SWITCH ON DISABLED with no fault, then recovers. Retrying with a short,
    growing wait lets a later attempt land after the blip has passed, so a
    transient can't stop the line. A real fault, a non-enable failure, a stop
    request, or exhausting the attempts returns the failure untouched.

    Returns ``(JobResult, reenables)``."""
    res = run_job()
    reenables = 0
    delay = base_delay
    for _ in range(attempts):
        if res.ok:
            break
        if getattr(driver, "is_faulted", False):
            break                                  # a real fault — don't paper over it
        if "disabled" not in res.reason.lower():
            break                                  # some other failure — not our case
        if should_stop is not None and should_stop():
            break
        time.sleep(delay)                          # let the transient dip pass
        try:
            driver.enable()
        except Exception as exc:                   # noqa: BLE001 - keep retrying
            res = JobResult(False, f"{res.reason}; re-enable failed: {exc}")
            delay = min(delay * 2, 0.5)
            continue
        reenables += 1
        res = run_job()
        delay = min(delay * 2, 0.5)
    return res, reenables


def run_demo_cycle(
    controller: RobotTestController,
    pick_xy: Point,
    drops: List[Point],
    *,
    pick_sequence: Optional[PickSequence] = None,
    move_speed_mm_s: Optional[float] = DEMO_MOVE_SPEED_MM_S,
    runner: Optional[JobRunner] = None,
    should_stop: Optional[Callable[[], bool]] = None,
    on_step: Optional[Callable[[CycleStep], None]] = None,
    auto_reenable: bool = True,
    reenable_attempts: int = REENABLE_ATTEMPTS,
    reenable_delay_s: float = REENABLE_DELAY_S,
) -> CycleResult:
    """Drive a fixed-pick -> each-drop pick&place with full head actuation.

    The bench demo (vision bypass): every pick is the same supply ``pick_xy``
    nest, every drop is one hole of the cover row. Unlike ``run_cycle`` there is
    no camera, detection, or cover-selection — just plan -> validate -> move for
    each hole in turn. Enforces a motion preflight (enabled + referenced) so it
    can never move an unhomed robot. The gate reads the driver's live state (the
    same source the bench Set Home / jog use), not the controller's home cache."""
    driver = controller.driver
    if not driver.is_enabled:
        return CycleResult(ok=False, reason="drives are disabled — Enable them first")
    if not driver.is_referenced:
        return CycleResult(ok=False, reason="robot is not referenced — Set Home first")
    runner = runner or make_job_runner(driver, pick_sequence,
                                       move_speed_mm_s=move_speed_mm_s)
    reenables = 0
    kin, validator = controller.kin, controller.validator
    result = CycleResult()
    for i, drop_xy in enumerate(drops):
        if should_stop is not None and should_stop():
            result.reason = "stopped by operator"
            break
        try:
            job = make_job(kin, validator, hole_index=i, cover_id=i,
                           pick_xy=pick_xy, drop_xy=drop_xy)
        except PlanningError as exc:
            step = CycleStep(i, drop_xy, i, pick_xy, False, str(exc))
            result.steps.append(step)
            if on_step is not None:
                on_step(step)
            continue
        if auto_reenable:
            # A drive can fall back out of Operation Enabled WITHOUT a fault —
            # a power-stage enable-chain blip (E-stop contactor chatter, a bus-
            # power dip; the AS715N has no STO, so its stage feeds through the
            # contactor). Re-arm and retry with growing backoff so a later
            # attempt lands after the dip passes; a real fault, a stop request,
            # or exhausting the retries still aborts below.
            res, n = _run_with_reenable(
                driver, lambda: runner.run(job), should_stop=should_stop,
                attempts=reenable_attempts, base_delay=reenable_delay_s)
            if n:
                reenables += n
                logger.warning(
                    "demo: drive dropped out mid-run — re-enabled %dx (%s)",
                    n, "recovered" if res.ok else "still failing")
        else:
            res = runner.run(job)
        step = CycleStep(i, drop_xy, i, pick_xy, res.ok, res.reason)
        result.steps.append(step)
        if on_step is not None:
            on_step(step)
        if not res.ok:
            result.reason = f"job failed: {res.reason}"
            return result
    placed = len(result.placed)
    if not result.reason:
        result.reason = f"placed {placed}/{len(drops)} covers"
    if reenables:
        result.reason += (f" [auto re-enabled {reenables}x after an unexpected "
                          f"drive disable — check the STO/E-stop chain]")
    result.ok = placed > 0
    return result


class CycleRateTracker:
    """Rolling pick&place throughput, in completed cycles per minute.

    One placed cover = one cycle. Feed monotonically-increasing timestamps (in
    seconds, e.g. ``time.perf_counter()``); ``per_minute`` reports the rate over
    a trailing window (default 60 s), or over the elapsed time until the window
    first fills, so the reading is meaningful from the second cycle on."""

    def __init__(self, window_s: float = 60.0) -> None:
        self.window_s = float(window_s)
        self._stamps: Deque[float] = deque()
        self._start: Optional[float] = None
        self.total = 0

    def record(self, t: float) -> None:
        if self._start is None:
            self._start = t
        self._stamps.append(t)
        self.total += 1
        self._prune(t)

    def _prune(self, t: float) -> None:
        while self._stamps and t - self._stamps[0] > self.window_s:
            self._stamps.popleft()

    def per_minute(self, t: Optional[float] = None) -> float:
        """Cycles/min as of time ``t`` (defaults to the last recorded time)."""
        if self._start is None:
            return 0.0
        if t is None:
            t = self._stamps[-1] if self._stamps else self._start
        self._prune(t)
        elapsed = min(t - self._start, self.window_s)
        if elapsed <= 0.0:
            return 0.0
        return len(self._stamps) * 60.0 / elapsed


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
        pick_sequence: Optional[PickSequence] = None,
    ) -> None:
        self.controller = controller
        self.camera = camera
        # One plane for both holes and covers; changeover swaps this per battery.
        self.calibration = calibration
        self.hole_detector = hole_detector or HoleDetector()
        self.cover_detector = cover_detector or CoverDetector()
        self.job_runner = job_runner
        self.pick_sequence = pick_sequence
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
            return "drives are disabled — Enable them on the Drives tab first"
        if not self.controller.is_referenced:
            return "robot is not referenced — Set Home on the Drives tab first"
        return None

    def preflight(self) -> Optional[str]:
        """Return a blocking reason, or None if the cycle may run."""
        return self._motion_block() or self.target_source.preflight()

    # --- run ----------------------------------------------------------------
    def run_cycle(
        self,
        should_stop: Optional[Callable[[], bool]] = None,
        on_step: Optional[Callable[[CycleStep], None]] = None,
        on_frame: Optional[Callable[[object], None]] = None,
    ) -> CycleResult:
        block = self.preflight()
        if block is not None:
            return CycleResult(ok=False, reason=block)

        runner = self.job_runner or make_job_runner(
            self.controller.driver, self.pick_sequence)
        kin, validator = self.controller.kin, self.controller.validator

        # Holes -> ordered drop targets (robot frame), from vision or a bypass.
        try:
            hole_xy = self.target_source.holes()
        except CameraError as exc:
            return CycleResult(ok=False, reason=f"capture failed: {exc}")
        self._emit_frame(on_frame)
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
            self._emit_frame(on_frame)          # live-update the view on each re-image
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
                return result  # a drive fault/timeout stops the cycle

        placed = len(result.placed)
        if not result.reason:
            result.reason = f"placed {placed}/{len(order)} covers"
        result.ok = placed > 0
        return result

    def _emit_frame(self, on_frame: Optional[Callable[[object], None]]) -> None:
        """Hand the most recently imaged frame to the GUI so the vision screen
        refreshes on every capture during the run (no-op for a source-less bypass)."""
        if on_frame is None:
            return
        frame = getattr(self.target_source, "last_frame", None)
        if frame is not None:
            on_frame(frame)

    # --- single job (vision + selection bypass) -----------------------------
    def run_single_job(self, pick_xy: Point, drop_xy: Point) -> CycleStep:
        """Fire ONE pick->place job from explicit robot-frame points. No vision,
        no cover selection — the simplest way to exercise IK + validation + a
        coordinated move on hardware."""
        block = self._motion_block()
        if block is not None:
            return CycleStep(0, drop_xy, 0, pick_xy, False, block)
        runner = self.job_runner or make_job_runner(
            self.controller.driver, self.pick_sequence)
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
