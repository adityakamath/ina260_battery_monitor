"""Tests for BatteryETA: EMA smoothing and time-to-full/time-to-empty estimation."""

import pytest
from ina260_battery_monitor.battery_eta import BatteryETA

_DESIGN_CAPACITY_AH = 5.0


def _eta(**overrides):
    kwargs = dict(idle_current_threshold_a=0.05, current_ema_alpha=1.0)
    kwargs.update(overrides)
    return BatteryETA(**kwargs)


class TestIdle:

    def test_no_reading_yet_returns_none_none(self):
        eta = _eta()
        assert eta.estimate(charge_ah=2.5, design_capacity_ah=_DESIGN_CAPACITY_AH) == (None, None)

    def test_current_within_idle_threshold_returns_none_none(self):
        eta = _eta(idle_current_threshold_a=0.05)
        eta.update(0.02)
        assert eta.estimate(charge_ah=2.5, design_capacity_ah=_DESIGN_CAPACITY_AH) == (None, None)


class TestCharging:

    def test_time_to_full_from_remaining_capacity(self):
        eta = _eta()
        eta.update(1.0)  # 1 A in, alpha=1.0 -> smoothed_current_a = 1.0 exactly
        time_to_full_s, time_to_empty_s = eta.estimate(
            charge_ah=2.5, design_capacity_ah=_DESIGN_CAPACITY_AH
        )
        assert time_to_empty_s is None
        # 2.5 Ah remaining / 1.0 A = 2.5 h = 9000 s
        assert time_to_full_s == pytest.approx(9000.0)

    def test_already_full_gives_zero_time_to_full(self):
        eta = _eta()
        eta.update(1.0)
        time_to_full_s, _ = eta.estimate(
            charge_ah=_DESIGN_CAPACITY_AH, design_capacity_ah=_DESIGN_CAPACITY_AH
        )
        assert time_to_full_s == pytest.approx(0.0)


class TestDischarging:

    def test_time_to_empty_from_remaining_charge(self):
        eta = _eta()
        eta.update(-0.5)
        time_to_full_s, time_to_empty_s = eta.estimate(
            charge_ah=2.5, design_capacity_ah=_DESIGN_CAPACITY_AH
        )
        assert time_to_full_s is None
        # 2.5 Ah / 0.5 A = 5 h = 18000 s
        assert time_to_empty_s == pytest.approx(18000.0)


class TestSmoothing:

    def test_partial_alpha_moves_toward_new_reading_gradually(self):
        eta = _eta(current_ema_alpha=0.5)
        eta.update(1.0)
        assert eta.smoothed_current_a == pytest.approx(1.0)
        eta.update(0.0)
        assert eta.smoothed_current_a == pytest.approx(0.5)
        eta.update(0.0)
        assert eta.smoothed_current_a == pytest.approx(0.25)
