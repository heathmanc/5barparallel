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
from .constants import (
    COMMISSIONING_CONSTANTS,
    PlcConstant,
    PlcConstantStore,
    PushResult,
    default_values,
    push_constants,
    read_constants,
)
from .handshake import JobResult, PickPlaceHandshake
from .plc_robot_driver import PlcRobotDriver
from .tags import TAG_SPECS, TagSpec, tag_table_csv

__all__ = [
    "COMMISSIONING_CONSTANTS",
    "CompactLogixClient",
    "JobResult",
    "PickPlaceHandshake",
    "PlcClient",
    "PlcConstant",
    "PlcConstantStore",
    "PlcError",
    "PlcRobotDriver",
    "PushResult",
    "SimulatedPlcClient",
    "TAG_SPECS",
    "TagSpec",
    "default_values",
    "push_constants",
    "read_constants",
    "tag_table_csv",
    "tags",
]
