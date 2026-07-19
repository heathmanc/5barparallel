# CLAUDE.md — working notes for AI sessions

Vision-guided 5-bar parallel-SCARA bung-cover pick-and-place robot.
Python + PySide6 HMI + machine vision + EtherCAT servo control (StepperOnline
A6-EC / AS715N drives over the IgH EtherLab master).

## Git workflow (standing preference)

- Develop on the designated feature branch; commit with clear messages.
- **Auto-sync on push.** If a push is rejected because the remote branch moved,
  pull and merge (`git pull --no-rebase`), resolve, and re-push automatically —
  no manual commands handed to the user, no pausing to ask.
- **Merge to `main` when a unit of work is complete.** Open a PR from the
  feature branch and merge it into `main` (the default branch) once the work is
  finished and the test suite is green. Don't wait to be told each time.
- After a branch has been merged, treat follow-up work as a fresh change:
  restart the branch from the latest `main` rather than stacking new commits on
  already-merged history.

## Tests / build

- `python -m pytest -q` — full suite (should stay green before any merge).
- `python cad/generate.py` — regenerate STEP models + `docs/cad/*.png`
  (needs `cadquery`).
- `make -C igh ETHERLAB=/opt/etherlab` — build the IgH RT daemon (rebuild
  whenever the shared-memory ABI in `igh/ec_master_daemon.c` changes).
