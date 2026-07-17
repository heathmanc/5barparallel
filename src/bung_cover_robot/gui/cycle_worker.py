"""Runs a CycleManager off the GUI thread.

A pick/place job takes seconds per hole (coordinated moves + vacuum settle), so
``run_cycle`` must not run on the Qt main thread or the HMI would freeze. This QObject is moved to a
QThread; it emits ``stepDone`` after each hole and ``finished`` with the
CycleResult. ``request_stop`` sets a flag the manager checks *between* holes, so
Stop lets the in-flight pick finish safely (never aborts mid-transfer, which
could drop a cover) before halting.
"""

from __future__ import annotations

from PySide6.QtCore import QObject, Signal

from ..app.cycle_manager import CycleManager


class CycleWorker(QObject):
    stepDone = Signal(object)    # CycleStep, emitted per hole
    frameReady = Signal(object)  # np.ndarray, emitted on every camera capture
    finished = Signal(object)    # CycleResult, emitted once at the end

    def __init__(self, manager: CycleManager) -> None:
        super().__init__()
        self._manager = manager
        self._stop = False

    def request_stop(self) -> None:
        """Ask the cycle to halt after the current pick (thread-safe flag)."""
        self._stop = True

    def run(self) -> None:
        result = self._manager.run_cycle(
            should_stop=lambda: self._stop,
            on_step=self.stepDone.emit,
            on_frame=self.frameReady.emit,
        )
        self.finished.emit(result)
