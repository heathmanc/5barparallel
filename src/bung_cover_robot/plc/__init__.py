"""PLC layer — EtherNet/IP to the CompactLogix (Claude.md §11, §14).

Tag constants, a pycomm3 client + reactive simulator (manual jog/home *and* the
automatic pick/place handshake), a PLC-backed RobotDriver for manual jog/home,
and the automatic pick/place handshake (handshake.py).
"""

from . import tags
from .compactlogix_client import (
    CompactLogixClient,
    PlcClient,
    PlcError,
    SimulatedPlcClient,
)
from .handshake import JobResult, PickPlaceHandshake
from .plc_robot_driver import PlcRobotDriver
from .tags import TAG_SPECS, TagSpec, tag_table_csv

__all__ = [
    "CompactLogixClient",
    "JobResult",
    "PickPlaceHandshake",
    "PlcClient",
    "PlcError",
    "PlcRobotDriver",
    "SimulatedPlcClient",
    "TAG_SPECS",
    "TagSpec",
    "tag_table_csv",
    "tags",
]
