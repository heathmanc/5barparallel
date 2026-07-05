"""PLC layer — EtherNet/IP to the CompactLogix (Claude.md §11, §14).

Implemented so far: tag constants, a pycomm3 client + reactive simulator, and a
PLC-backed RobotDriver for manual jog/home. The automatic pick/place handshake
(handshake.py) is still to build.
"""

from . import tags
from .compactlogix_client import (
    CompactLogixClient,
    PlcClient,
    PlcError,
    SimulatedPlcClient,
)
from .plc_robot_driver import PlcRobotDriver
from .tags import TAG_SPECS, TagSpec, tag_table_csv

__all__ = [
    "CompactLogixClient",
    "PlcClient",
    "PlcError",
    "PlcRobotDriver",
    "SimulatedPlcClient",
    "TAG_SPECS",
    "TagSpec",
    "tag_table_csv",
    "tags",
]
