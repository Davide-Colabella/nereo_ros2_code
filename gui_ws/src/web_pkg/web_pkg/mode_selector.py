"""
Guidance mode selector: MANUAL (pilot) or AUTONOMOUS (mission bridge).

Pure logic, no ROS, so it can be unit-tested; safety_node owns the ROS side.
Rules (decision 7 of the thesis mission software, Tesi-Magistrale
ros2/proposta_adattatore.md):

  - start in MANUAL;
  - the override (a joystick button, or the manual request service) always
    goes back to MANUAL, whatever the mission state;
  - AUTONOMOUS only on an explicit request, and only when the mission state
    is known, fresh and neither Mission nor Abort;
  - in AUTONOMOUS motion commands and arm requests come only from the mission
    bridge; in MANUAL only from the pilot side (joystick, GUI, web).

Mode values are those of /modalita_guida (std_msgs/UInt8), read by
nereo_mission_bridge: 0 MANUAL, 1 AUTONOMOUS.
"""

MANUAL = 0
AUTONOMOUS = 1

# Top-level states of the mission chart (mission_status.state)
MISSION_STATE_MISSION = 3
MISSION_STATE_ABORT = 4
MISSION_STATE_NAMES = {
    0: 'Boot', 1: 'Ready', 2: 'PreDive', 3: 'Mission', 4: 'Abort',
    5: 'Surfaced', 6: 'FaultSafe', 7: 'Shutdown',
}

# Command sources
SOURCE_PILOT = 'pilot'    # joystick (direct and through the controller), GUI, web
SOURCE_BRIDGE = 'bridge'  # nereo_mission_bridge


def mode_name(mode):
    return 'AUTONOMOUS' if mode == AUTONOMOUS else 'MANUAL'


class ModeSelector:
    def __init__(self, mission_state_timeout=2.0):
        self.mode = MANUAL
        self.mission_state_timeout = mission_state_timeout
        self._mission_state = None
        self._mission_state_t = None
        self._buttons = {}

    # ── inputs ────────────────────────────────────────────────────────────

    def set_mission_state(self, state, t):
        self._mission_state = int(state)
        self._mission_state_t = t

    def mission_state(self, t):
        """Mission state, or None when unknown or older than the timeout."""
        if self._mission_state is None:
            return None
        if t - self._mission_state_t > self.mission_state_timeout:
            return None
        return self._mission_state

    # ── requests ──────────────────────────────────────────────────────────

    def override(self):
        """Back to MANUAL from any state. Returns True if the mode changed."""
        changed = self.mode != MANUAL
        self.mode = MANUAL
        return changed

    def request_autonomous(self, t):
        """Explicit request for AUTONOMOUS. Returns (accepted, reason)."""
        if self.mode == AUTONOMOUS:
            return True, 'already AUTONOMOUS'
        state = self.mission_state(t)
        if state is None:
            return False, 'mission state unknown or stale'
        if state in (MISSION_STATE_MISSION, MISSION_STATE_ABORT):
            return False, 'mission in %s' % MISSION_STATE_NAMES[state]
        self.mode = AUTONOMOUS
        return True, 'AUTONOMOUS (mission in %s)' % MISSION_STATE_NAMES.get(state, state)

    def on_buttons(self, buttons, override_button, autonomous_button, t):
        """
        Joystick buttons (sensor_msgs/Joy.buttons). Acts on rising edges; the
        override wins over an autonomous request in the same message.
        Returns a list of (event, accepted, reason) for logging.
        """
        events = []
        pressed = self._rising(buttons, override_button)
        if pressed:
            changed = self.override()
            events.append(('override', True, 'MANUAL' if changed else 'already MANUAL'))
        elif self._rising(buttons, autonomous_button):
            ok, reason = self.request_autonomous(t)
            events.append(('autonomous', ok, reason))
        # remember the state of the other button too, so a press held during
        # the override does not fire later
        for b in (override_button, autonomous_button):
            if b is not None and b >= 0:
                self._buttons[b] = self._button(buttons, b)
        return events

    # ── routing ───────────────────────────────────────────────────────────

    def accepts(self, source):
        """Whether motion commands and arm requests from source pass."""
        if self.mode == AUTONOMOUS:
            return source == SOURCE_BRIDGE
        return source == SOURCE_PILOT

    # ── helpers ───────────────────────────────────────────────────────────

    @staticmethod
    def _button(buttons, b):
        return 1 if 0 <= b < len(buttons) and buttons[b] else 0

    def _rising(self, buttons, b):
        if b is None or b < 0:
            return False
        now = self._button(buttons, b)
        before = self._buttons.get(b, 0)
        self._buttons[b] = now
        return now == 1 and before == 0
