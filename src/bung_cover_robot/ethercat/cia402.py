"""CiA 402 drive profile — the state machine every servo drive speaks.

The StepperOnline A6 EtherCAT drives are CoE (CANopen-over-EtherCAT) servo drives
implementing the CiA 402 device profile. Motion is controlled through two 16-bit
words exchanged as cyclic PDOs:

  * Controlword (0x6040)  — PC -> drive: commands the drive's state transitions.
  * Statusword (0x6041)   — drive -> PC: reports the drive's current state.

plus a Modes-of-operation byte (0x6060) that selects CSP / Homing / etc.

To make a drive move you must walk it up the CiA 402 state machine to
``OPERATION_ENABLED``:

    SWITCH_ON_DISABLED --Shutdown--> READY_TO_SWITCH_ON --SwitchOn--> SWITCHED_ON
        --EnableOperation--> OPERATION_ENABLED

A fault drops the drive to ``FAULT``; a rising edge of controlword bit 7 (Fault
Reset) clears it back to ``SWITCH_ON_DISABLED``.

This module is pure logic — no EtherCAT, no I/O — so the whole arming/fault
sequence is unit-testable with nothing connected. ``next_controlword`` is the
single helper the driver calls each cycle: give it the live statusword and it
returns the controlword that advances one step toward Operation Enabled (or
clears a fault first), so the driver never hard-codes the transition table.
"""

from __future__ import annotations

from enum import Enum

# --- Modes of operation (object 0x6060) ------------------------------------- #
MODE_PROFILE_POSITION = 1
MODE_PROFILE_VELOCITY = 3
MODE_HOMING = 6
MODE_CSP = 8            # Cyclic Synchronous Position — the mode we run in
MODE_CSV = 9           # Cyclic Synchronous Velocity


# --- Statusword bit masks (object 0x6041) ----------------------------------- #
SW_READY_TO_SWITCH_ON = 1 << 0
SW_SWITCHED_ON = 1 << 1
SW_OPERATION_ENABLED = 1 << 2
SW_FAULT = 1 << 3
SW_VOLTAGE_ENABLED = 1 << 4
SW_QUICK_STOP = 1 << 5
SW_SWITCH_ON_DISABLED = 1 << 6
SW_WARNING = 1 << 7
SW_TARGET_REACHED = 1 << 10
SW_INTERNAL_LIMIT = 1 << 11
# Homing-mode-specific (CiA 402-2): bit 12 = homing attained, bit 13 = homing error.
SW_HOMING_ATTAINED = 1 << 12
SW_HOMING_ERROR = 1 << 13


# --- Controlword commands (object 0x6040) ----------------------------------- #
CW_SHUTDOWN = 0x0006          # -> READY_TO_SWITCH_ON
CW_SWITCH_ON = 0x0007         # -> SWITCHED_ON
CW_ENABLE_OPERATION = 0x000F  # -> OPERATION_ENABLED
CW_DISABLE_VOLTAGE = 0x0000
CW_QUICK_STOP = 0x0002
CW_FAULT_RESET = 0x0080       # rising edge of bit 7 clears a fault
# Homing-mode operation start: bit 4 (start homing) on top of Enable Operation.
CW_HOMING_START = 0x001F


class Cia402State(Enum):
    """The CiA 402 drive states, decoded from the statusword."""

    NOT_READY_TO_SWITCH_ON = "not_ready_to_switch_on"
    SWITCH_ON_DISABLED = "switch_on_disabled"
    READY_TO_SWITCH_ON = "ready_to_switch_on"
    SWITCHED_ON = "switched_on"
    OPERATION_ENABLED = "operation_enabled"
    QUICK_STOP_ACTIVE = "quick_stop_active"
    FAULT_REACTION_ACTIVE = "fault_reaction_active"
    FAULT = "fault"


def decode_state(statusword: int) -> Cia402State:
    """Map a raw statusword to its CiA 402 state.

    The state lives in statusword bits 0-3, 5, 6; each state has a defining bit
    pattern under a mask (per CiA 402 Table). Order matters: fault patterns are
    checked with the wider 0x4F mask (quick-stop bit is don't-care) before the
    running states with the 0x6F mask.
    """
    w = statusword & 0xFFFF
    if (w & 0x4F) == 0x00:
        return Cia402State.NOT_READY_TO_SWITCH_ON
    if (w & 0x4F) == 0x40:
        return Cia402State.SWITCH_ON_DISABLED
    if (w & 0x4F) == 0x0F:
        return Cia402State.FAULT_REACTION_ACTIVE
    if (w & 0x4F) == 0x08:
        return Cia402State.FAULT
    if (w & 0x6F) == 0x21:
        return Cia402State.READY_TO_SWITCH_ON
    if (w & 0x6F) == 0x23:
        return Cia402State.SWITCHED_ON
    if (w & 0x6F) == 0x27:
        return Cia402State.OPERATION_ENABLED
    if (w & 0x6F) == 0x07:
        return Cia402State.QUICK_STOP_ACTIVE
    # Unknown/transitional word — treat as not-ready so the caller re-drives it.
    return Cia402State.NOT_READY_TO_SWITCH_ON


def is_fault(statusword: int) -> bool:
    return decode_state(statusword) in (
        Cia402State.FAULT,
        Cia402State.FAULT_REACTION_ACTIVE,
    )


def is_operation_enabled(statusword: int) -> bool:
    return decode_state(statusword) is Cia402State.OPERATION_ENABLED


def next_controlword(statusword: int, prev_controlword: int = 0) -> int:
    """The controlword that advances the drive one step toward OPERATION_ENABLED.

    Call every cycle with the live statusword; feed back the value you last wrote
    as ``prev_controlword`` so a Fault Reset is issued as a proper rising edge on
    bit 7 (assert once, then drop it) rather than being held. When already in
    OPERATION_ENABLED this returns ``CW_ENABLE_OPERATION`` (a no-op hold), so it's
    safe to call unconditionally.

    Drives one transition per call by design: PDO-synchronous state machines need
    the drive to observe each intermediate state, so the driver polls the
    statusword between steps rather than blasting 0x0F immediately.
    """
    state = decode_state(statusword)
    if state in (Cia402State.FAULT, Cia402State.FAULT_REACTION_ACTIVE):
        # Fault Reset is edge-triggered: only assert bit 7 if we didn't last time.
        if prev_controlword & CW_FAULT_RESET:
            return prev_controlword & ~CW_FAULT_RESET  # drop it to re-arm the edge
        return CW_FAULT_RESET
    if state is Cia402State.SWITCH_ON_DISABLED:
        return CW_SHUTDOWN
    if state is Cia402State.READY_TO_SWITCH_ON:
        return CW_SWITCH_ON
    if state is Cia402State.SWITCHED_ON:
        return CW_ENABLE_OPERATION
    if state is Cia402State.OPERATION_ENABLED:
        return CW_ENABLE_OPERATION
    if state is Cia402State.QUICK_STOP_ACTIVE:
        # Leave quick-stop by disabling voltage, then re-arm from the top.
        return CW_DISABLE_VOLTAGE
    # NOT_READY_TO_SWITCH_ON (or unknown): the drive is booting; hold shutdown.
    return CW_SHUTDOWN
