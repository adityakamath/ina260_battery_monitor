"""Charger-presence detection from /battery_state current alone - no GPIO.

A single net-current sensor can't tell "charger connected but outpaced by load" apart from
"no charger" under arbitrary motion - the two currents just add, and any net reading is
consistent with many different (charger, load) pairs (see battery_events_node.py's module
docstring). But when the robot is genuinely holding still, load collapses to the board's
own quiescent draw, which is small and roughly constant for a given robot - so a resting
current measurably above a calibrated no-charger baseline is a real, checkable signal.
Bench-measured on a real LeKiwi (2026-09-13): -0.60 A resting with the charger connected
vs. -1.72 A without, a clean 1.12 A gap.

"Holding still" is inferred here from the current signal's own stability, not motion data
(none is available at this layer): if current has stayed within a narrow spread for
rest_window_samples consecutive readings, that's flat enough to trust as a rest reading. A
wider spread means the robot is (or was, recently) doing something - the detector holds its
last determination rather than guessing off a driving reading. This is a deliberate
tradeoff, not an oversight: bench-testing while actually driving (2026-09-13) showed the
charger signal can still separate cleanly under light load (-1.51 A vs +0.88 A that
session), but there is no guarantee heavier/sustained load couldn't exceed the charger's
own output and push net current negative even with it connected - only a rest reading is
provably decidable from this one sensor, so only a rest reading is trusted here.
"""

from collections import deque


class ChargerPresenceDetector:

    def __init__(
        self,
        no_charger_idle_current_a: float,
        charger_current_margin_a: float,
        rest_spread_threshold_a: float,
        rest_window_samples: int,
        hysteresis_a: float = 0.2,
        flip_confirm_samples: int = 40,
    ):
        self._threshold_a = no_charger_idle_current_a + charger_current_margin_a
        self._hysteresis_a = hysteresis_a
        self._flip_confirm_samples = flip_confirm_samples
        self._flip_streak = 0
        self._rest_spread_threshold_a = rest_spread_threshold_a
        self._window = deque(maxlen=rest_window_samples)
        self._connected = None  # None = no determination made yet

    @property
    def connected(self):
        return self._connected

    def update(self, current_a: float):
        """Fold in one new current reading.

        Returns the new connected state (True/False) the instant it changes, else None -
        callers that only want to act on genuine edges (e.g. driving a SetBool service)
        can treat None as "nothing to do", matching this package's other watchers.
        """
        self._window.append(current_a)
        if len(self._window) < self._window.maxlen:
            return None

        if max(self._window) - min(self._window) > self._rest_spread_threshold_a:
            self._flip_streak = 0
            return None  # not settled - hold the last determination

        mean_current = sum(self._window) / len(self._window)
        if self._connected is None:
            self._connected = mean_current > self._threshold_a
            return self._connected

        # Flipping needs to clear the threshold by the hysteresis band and stay there for
        # flip_confirm_samples settled readings, so a reading hovering near the threshold
        # can't toggle (and re-announce) repeatedly.
        if self._connected:
            wants_flip = mean_current < self._threshold_a - self._hysteresis_a
        else:
            wants_flip = mean_current > self._threshold_a + self._hysteresis_a
        self._flip_streak = self._flip_streak + 1 if wants_flip else 0
        if self._flip_streak < self._flip_confirm_samples:
            return None
        self._flip_streak = 0
        self._connected = not self._connected
        return self._connected
