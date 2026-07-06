"""Allow ``python -m bung_cover_robot`` to launch the HMI."""

from .main import main

if __name__ == "__main__":
    raise SystemExit(main())
