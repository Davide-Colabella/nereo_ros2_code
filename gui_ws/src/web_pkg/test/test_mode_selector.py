"""Unit tests of the guidance mode selector (web_pkg.mode_selector), no ROS."""
import pytest

from web_pkg.mode_selector import (
    AUTONOMOUS, MANUAL, MISSION_STATE_ABORT, MISSION_STATE_MISSION, ModeSelector,
    SOURCE_BRIDGE, SOURCE_PILOT)

ALL_MISSION_STATES = list(range(8))   # Boot .. Shutdown
ALLOWED_STATES = [s for s in ALL_MISSION_STATES
                  if s not in (MISSION_STATE_MISSION, MISSION_STATE_ABORT)]
OVERRIDE = 1
AUTO = 3


def buttons(*pressed, n=12):
    return [1 if i in pressed else 0 for i in range(n)]


def selector_in(mode, mission_state=1, t=0.0):
    """A selector in the given mode, with a fresh mission state."""
    s = ModeSelector(mission_state_timeout=2.0)
    s.set_mission_state(1, t)
    if mode == AUTONOMOUS:
        assert s.request_autonomous(t)[0]
    s.set_mission_state(mission_state, t)
    return s


# ── start ─────────────────────────────────────────────────────────────────

def test_starts_in_manual():
    s = ModeSelector()
    assert s.mode == MANUAL
    assert s.accepts(SOURCE_PILOT)
    assert not s.accepts(SOURCE_BRIDGE)


# ── override ──────────────────────────────────────────────────────────────

@pytest.mark.parametrize('mode', [MANUAL, AUTONOMOUS])
@pytest.mark.parametrize('mission_state', ALL_MISSION_STATES + [None])
def test_override_goes_back_to_manual_from_every_state(mode, mission_state):
    s = selector_in(mode, 1)
    if mission_state is None:
        s._mission_state = None   # mission state lost
    else:
        s.set_mission_state(mission_state, 0.0)
    changed = s.override()
    assert s.mode == MANUAL
    assert changed == (mode == AUTONOMOUS)
    assert s.accepts(SOURCE_PILOT) and not s.accepts(SOURCE_BRIDGE)


@pytest.mark.parametrize('mission_state', ALL_MISSION_STATES)
def test_override_button_from_every_state(mission_state):
    s = selector_in(AUTONOMOUS, mission_state)
    events = s.on_buttons(buttons(OVERRIDE), OVERRIDE, AUTO, 0.1)
    assert s.mode == MANUAL
    assert events == [('override', True, 'MANUAL')]


def test_override_wins_over_autonomous_in_the_same_message():
    s = selector_in(MANUAL, 1)
    s.on_buttons(buttons(OVERRIDE, AUTO), OVERRIDE, AUTO, 0.1)
    assert s.mode == MANUAL
    # the autonomous button still held after the override does not fire
    s.on_buttons(buttons(AUTO), OVERRIDE, AUTO, 0.2)
    assert s.mode == MANUAL


def test_buttons_act_on_rising_edges_only():
    s = selector_in(MANUAL, 1)
    s.on_buttons(buttons(AUTO), OVERRIDE, AUTO, 0.1)
    assert s.mode == AUTONOMOUS
    s.on_buttons(buttons(OVERRIDE), OVERRIDE, AUTO, 0.2)
    assert s.mode == MANUAL
    # auto button released and pressed again: new request
    s.on_buttons(buttons(), OVERRIDE, AUTO, 0.3)
    s.on_buttons(buttons(AUTO), OVERRIDE, AUTO, 0.4)
    assert s.mode == AUTONOMOUS
    # override held: no repeated events
    s.on_buttons(buttons(OVERRIDE), OVERRIDE, AUTO, 0.5)
    assert s.on_buttons(buttons(OVERRIDE), OVERRIDE, AUTO, 0.6) == []


def test_autonomous_button_disabled_with_minus_one():
    s = selector_in(MANUAL, 1)
    assert s.on_buttons(buttons(*range(12)), OVERRIDE, -1, 0.1) == [
        ('override', True, 'already MANUAL')]
    assert s.mode == MANUAL


def test_short_joy_message_does_not_crash():
    s = selector_in(AUTONOMOUS, 1)
    s.on_buttons([], OVERRIDE, AUTO, 0.1)
    assert s.mode == AUTONOMOUS


# ── autonomous request ────────────────────────────────────────────────────

@pytest.mark.parametrize('mission_state', [MISSION_STATE_MISSION, MISSION_STATE_ABORT])
def test_autonomous_refused_in_mission_and_abort(mission_state):
    s = selector_in(MANUAL, mission_state)
    ok, reason = s.request_autonomous(0.1)
    assert not ok
    assert 'Mission' in reason or 'Abort' in reason
    assert s.mode == MANUAL
    assert not s.accepts(SOURCE_BRIDGE)


@pytest.mark.parametrize('mission_state', ALLOWED_STATES)
def test_autonomous_accepted_outside_mission_and_abort(mission_state):
    s = selector_in(MANUAL, mission_state)
    assert s.request_autonomous(0.1)[0]
    assert s.mode == AUTONOMOUS
    assert s.accepts(SOURCE_BRIDGE)
    assert not s.accepts(SOURCE_PILOT)


def test_autonomous_refused_without_mission_state():
    s = ModeSelector()
    ok, reason = s.request_autonomous(0.0)
    assert not ok and 'unknown' in reason
    assert s.mode == MANUAL


def test_autonomous_refused_with_stale_mission_state():
    s = ModeSelector(mission_state_timeout=2.0)
    s.set_mission_state(1, 0.0)
    assert not s.request_autonomous(2.5)[0]
    s.set_mission_state(1, 2.5)
    assert s.request_autonomous(2.6)[0]


def test_already_autonomous_stays_autonomous_even_in_mission():
    # entering Mission does not drop autonomy: only the override does
    s = selector_in(AUTONOMOUS, MISSION_STATE_MISSION)
    assert s.request_autonomous(0.1) == (True, 'already AUTONOMOUS')
    assert s.mode == AUTONOMOUS


def test_after_override_in_mission_autonomous_is_refused():
    s = selector_in(AUTONOMOUS, MISSION_STATE_MISSION)
    s.override()
    s.set_mission_state(MISSION_STATE_ABORT, 0.2)
    assert not s.request_autonomous(0.3)[0]
    assert s.mode == MANUAL
