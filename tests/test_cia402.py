"""CiA 402 state-machine decode + arming/fault-reset transitions."""

from bung_cover_robot.ethercat.cia402 import (
    CW_ENABLE_OPERATION,
    CW_FAULT_RESET,
    CW_SHUTDOWN,
    CW_SWITCH_ON,
    Cia402State,
    decode_state,
    is_fault,
    is_operation_enabled,
    next_controlword,
)

# Representative statuswords for each state (defining bits under the mask, with
# voltage-enabled bit 4 set as a real drive reports it).
SW = {
    Cia402State.SWITCH_ON_DISABLED: 0x0040,
    Cia402State.READY_TO_SWITCH_ON: 0x0021,
    Cia402State.SWITCHED_ON: 0x0023,
    Cia402State.OPERATION_ENABLED: 0x0027,
    Cia402State.QUICK_STOP_ACTIVE: 0x0007,
    Cia402State.FAULT: 0x0008,
    Cia402State.FAULT_REACTION_ACTIVE: 0x000F,
    Cia402State.NOT_READY_TO_SWITCH_ON: 0x0000,
}


def test_decode_every_state():
    for state, word in SW.items():
        assert decode_state(word) is state


def test_decode_ignores_upper_status_bits():
    # target-reached (bit 10) / warning (bit 7) must not change the state decode.
    assert decode_state(0x0027 | (1 << 10) | (1 << 7)) is Cia402State.OPERATION_ENABLED


def test_fault_and_enabled_helpers():
    assert is_fault(SW[Cia402State.FAULT])
    assert is_fault(SW[Cia402State.FAULT_REACTION_ACTIVE])
    assert not is_fault(SW[Cia402State.OPERATION_ENABLED])
    assert is_operation_enabled(SW[Cia402State.OPERATION_ENABLED])
    assert not is_operation_enabled(SW[Cia402State.SWITCHED_ON])


def test_arming_walks_up_one_step_per_call():
    assert next_controlword(SW[Cia402State.SWITCH_ON_DISABLED]) == CW_SHUTDOWN
    assert next_controlword(SW[Cia402State.READY_TO_SWITCH_ON]) == CW_SWITCH_ON
    assert next_controlword(SW[Cia402State.SWITCHED_ON]) == CW_ENABLE_OPERATION
    # already enabled -> hold enable (safe no-op)
    assert next_controlword(SW[Cia402State.OPERATION_ENABLED]) == CW_ENABLE_OPERATION


def test_fault_reset_is_a_rising_edge():
    fault = SW[Cia402State.FAULT]
    # first call asserts bit 7...
    cw = next_controlword(fault, prev_controlword=0)
    assert cw & CW_FAULT_RESET
    # ...next call (still faulted, bit 7 was held) drops it to re-arm the edge.
    cw2 = next_controlword(fault, prev_controlword=cw)
    assert not (cw2 & CW_FAULT_RESET)


def test_full_arming_sequence_reaches_enabled():
    """Feed the controlword's implied next state back in until enabled."""
    # simulate a drive that advances one state per controlword it receives
    order = [
        Cia402State.SWITCH_ON_DISABLED,
        Cia402State.READY_TO_SWITCH_ON,
        Cia402State.SWITCHED_ON,
        Cia402State.OPERATION_ENABLED,
    ]
    idx = 0
    prev = 0
    for _ in range(10):
        word = SW[order[idx]]
        if is_operation_enabled(word):
            break
        cw = next_controlword(word, prev)
        prev = cw
        # a compliant drive: shutdown->ready, switchon->switched, enable->enabled
        if cw == CW_SHUTDOWN and order[idx] is Cia402State.SWITCH_ON_DISABLED:
            idx = 1
        elif cw == CW_SWITCH_ON and order[idx] is Cia402State.READY_TO_SWITCH_ON:
            idx = 2
        elif cw == CW_ENABLE_OPERATION and order[idx] is Cia402State.SWITCHED_ON:
            idx = 3
    assert order[idx] is Cia402State.OPERATION_ENABLED
