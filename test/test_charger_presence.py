"""Tests for ChargerPresenceDetector: rest-window spread gating + baseline threshold.

Calibration values below (idle_current_threshold, margin) match the LeKiwi bench
measurement in charger_presence.py's module docstring, so test cases read the same as
the real numbers rather than arbitrary ones.
"""

from ina260_battery_monitor.charger_presence import ChargerPresenceDetector

_NO_CHARGER_IDLE_A = -1.72
_MARGIN_A = 0.4  # threshold = -1.32
_SPREAD_THRESHOLD_A = 0.35
_WINDOW = 4


def _detector(**overrides):
    kwargs = dict(
        no_charger_idle_current_a=_NO_CHARGER_IDLE_A,
        charger_current_margin_a=_MARGIN_A,
        rest_spread_threshold_a=_SPREAD_THRESHOLD_A,
        rest_window_samples=_WINDOW,
    )
    kwargs.update(overrides)
    return ChargerPresenceDetector(**kwargs)


def _feed(detector, *readings):
    """Feed readings in order, returning the list of non-None (edge) results."""
    return [r for r in (detector.update(v) for v in readings) if r is not None]


class TestWindowFilling:

    def test_no_determination_before_window_full(self):
        detector = _detector()
        assert _feed(detector, -1.7, -1.7, -1.7) == []
        assert detector.connected is None


class TestRestDetection:

    def test_settled_no_charger_readings_declare_disconnected(self):
        detector = _detector()
        edges = _feed(detector, -1.75, -1.70, -1.72, -1.71)
        assert edges == [False]
        assert detector.connected is False

    def test_settled_charger_readings_declare_connected(self):
        detector = _detector()
        edges = _feed(detector, -0.60, -0.58, -0.62, -0.59)
        assert edges == [True]
        assert detector.connected is True

    def test_wide_spread_never_fires_a_determination(self):
        """Simulates driving: readings swing well past the spread threshold."""
        detector = _detector()
        edges = _feed(detector, -1.82, -1.25, -1.75, -1.30, -1.80, -1.28)
        assert edges == []
        assert detector.connected is None


class TestHoldingLastDetermination:

    def test_driving_after_a_rest_determination_holds_last_value(self):
        detector = _detector()
        _feed(detector, -1.75, -1.70, -1.72, -1.71)  # settles disconnected
        assert detector.connected is False

        # Now "driving": wide spread, must not change the determination.
        edges = _feed(detector, -1.25, -1.80, -1.30, -1.82)
        assert edges == []
        assert detector.connected is False

    def test_only_edges_are_reported_not_repeated_same_state(self):
        detector = _detector()
        _feed(detector, -1.75, -1.70, -1.72, -1.71)
        edges = _feed(detector, -1.73, -1.70, -1.72, -1.71)  # still settled, still disconnected
        assert edges == []


class TestTransition:

    def test_transitions_from_disconnected_to_connected_on_new_rest_window(self):
        detector = _detector(flip_confirm_samples=1)
        _feed(detector, -1.75, -1.70, -1.72, -1.71)
        assert detector.connected is False

        edges = _feed(detector, -0.60, -0.58, -0.62, -0.59, -0.60, -0.60, -0.60)
        assert edges == [True]
        assert detector.connected is True


class TestThreshold:

    def test_current_exactly_at_threshold_is_not_connected(self):
        """Threshold comparison is strict (>), so exactly-at-threshold stays disconnected."""
        detector = _detector()
        threshold = _NO_CHARGER_IDLE_A + _MARGIN_A
        edges = _feed(detector, threshold, threshold, threshold, threshold)
        assert edges == [False]

    def test_current_just_above_threshold_is_connected(self):
        detector = _detector()
        threshold = _NO_CHARGER_IDLE_A + _MARGIN_A
        edges = _feed(detector, threshold + 0.01, threshold + 0.01, threshold + 0.01, threshold + 0.01)
        assert edges == [True]


class TestFlipDebounce:

    def test_reading_near_threshold_does_not_flip(self):
        detector = _detector(flip_confirm_samples=2)
        _feed(detector, -1.75, -1.70, -1.72, -1.71)  # disconnected
        # -1.25 is above the -1.32 threshold but inside the hysteresis band
        assert _feed(detector, *([-1.25] * 20)) == []
        assert detector.connected is False

    def test_flip_requires_confirm_samples(self):
        detector = _detector(flip_confirm_samples=3)
        _feed(detector, -1.75, -1.70, -1.72, -1.71)
        edges = _feed(detector, *([-0.60] * 12))
        assert edges == [True]
        assert detector.connected is True
