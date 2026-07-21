#!/usr/bin/env bash
#
# install.sh — provision this project on a control PC (or a sim/office PC).
#
# Idempotent: safe to re-run after a `git pull` (rebuilds the IgH daemon, which
# you MUST do whenever its shared-memory ABI changes — the app refuses a stale
# one). Does the app-level work only; the OS-level prerequisites (PREEMPT_RT
# kernel, NIC pinned to ecat0, the IgH EtherLab master itself) are a one-time
# platform build documented in docs/deploy.md + igh/README.md.
#
# Usage:
#   scripts/install.sh                 # venv + app (gui+vision) + build daemon
#   scripts/install.sh --no-daemon     # sim/office PC: skip the daemon build
#   scripts/install.sh --etherlab DIR  # IgH install prefix (default /opt/etherlab)
#   scripts/install.sh --from-lock     # install pinned versions from requirements.lock
#   scripts/install.sh --test          # also run the test suite after installing
#   scripts/install.sh -h | --help
#
set -euo pipefail

ETHERLAB="${ETHERLAB:-/opt/etherlab}"
BUILD_DAEMON=1
FROM_LOCK=0
RUN_TEST=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-daemon) BUILD_DAEMON=0 ;;
        --etherlab)  ETHERLAB="${2:?--etherlab needs a path}"; shift ;;
        --from-lock) FROM_LOCK=1 ;;
        --test)      RUN_TEST=1 ;;
        -h|--help)   grep '^#' "$0" | grep -v '^#!' | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option: $1 (see --help)" >&2; exit 2 ;;
    esac
    shift
done

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

say()  { printf '\033[1;36m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m warn:\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m error:\033[0m %s\n' "$*" >&2; exit 1; }

# --- prerequisites ----------------------------------------------------------
say "Checking prerequisites"
command -v python3 >/dev/null || die "python3 not found"
PYV=$(python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])')
python3 -c 'import sys; sys.exit(0 if sys.version_info[:2] >= (3, 9) else 1)' \
    || die "Python >= 3.9 required (found $PYV)"
echo "    python $PYV"

if [[ $BUILD_DAEMON -eq 1 ]]; then
    command -v make >/dev/null || die "make not found (apt install build-essential)"
    command -v "${CC:-cc}" >/dev/null || command -v gcc >/dev/null \
        || die "no C compiler (apt install build-essential)"
    if [[ ! -e "$ETHERLAB/lib/libethercat.so" && ! -e "/usr/lib/libethercat.so" \
          && ! -e "$ETHERLAB/lib/libethercat.a" ]]; then
        warn "libethercat not found under $ETHERLAB/lib or /usr/lib."
        warn "Install the IgH EtherLab master first (see igh/README.md §1),"
        warn "or pass --etherlab <prefix>, or --no-daemon for a sim PC."
    fi
fi

# --- virtualenv -------------------------------------------------------------
if [[ ! -d .venv ]]; then
    say "Creating virtualenv (.venv)"
    python3 -m venv .venv
else
    say "Reusing existing virtualenv (.venv)"
fi
# shellcheck disable=SC1091
source .venv/bin/activate
python -m pip install --upgrade pip >/dev/null

# --- python deps ------------------------------------------------------------
if [[ $FROM_LOCK -eq 1 ]]; then
    [[ -f requirements.lock ]] || die "--from-lock but requirements.lock is missing"
    say "Installing pinned deps from requirements.lock"
    pip install -r requirements.lock
    pip install -e . --no-deps
else
    # gui = PySide6 (HMI), vision = headed OpenCV + pypylon (Basler). One source
    # of truth: the extras in pyproject.toml.
    say "Installing the app + GUI + vision deps (pip install -e .[gui,vision])"
    pip install -e ".[gui,vision]"
    say "Freezing an exact lockfile -> requirements.lock"
    pip freeze --exclude-editable > requirements.lock
    echo "    commit requirements.lock so the next PC installs identical versions"
    echo "    (re-create it with: scripts/install.sh --from-lock)"
fi

# --- IgH RT daemon ----------------------------------------------------------
if [[ $BUILD_DAEMON -eq 1 ]]; then
    say "Building the IgH RT daemon (make -C igh ETHERLAB=$ETHERLAB)"
    make -C igh ETHERLAB="$ETHERLAB"
    [[ -x igh/ec_master_daemon ]] || die "daemon did not build"
    echo "    built igh/ec_master_daemon"
else
    say "Skipping daemon build (--no-daemon)"
fi

# --- sanity -----------------------------------------------------------------
say "Import check"
python -c "import bung_cover_robot; print('    bung_cover_robot imports OK')"
if [[ $RUN_TEST -eq 1 ]]; then
    say "Running the test suite"
    pip install "pytest>=7.0" >/dev/null
    python -m pytest -q
fi

# --- next steps -------------------------------------------------------------
cat <<EOF

$(say "Done.")
Next steps for a REAL control PC (see docs/deploy.md for the full runbook):
  1. IgH master + NIC + RT kernel provisioned?  igh/README.md §1, ethercat_bringup.md §1
  2. Passwordless sudo for the daemon (the app auto-launches it with 'sudo -n'):
       docs/deploy.md → "Let the app launch the daemon"
  3. Copy your config/ (calibration, home, recipes) from the old PC.
  4. Dry-run first, then hardware:
       source .venv/bin/activate
       python -m bung_cover_robot --sim-ec        # no hardware, proves the stack
       python -m bung_cover_robot --ethercat --camera basler
EOF
