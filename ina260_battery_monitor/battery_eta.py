"""EMA-smoothed current -> time-to-full / time-to-empty estimate.

Raw instantaneous current is noisy (motor draw spikes, I2C read jitter), so dividing
charge_ah directly by a single reading would jitter the ETA wildly between publishes.
Smoothing current with an EMA first gives a stable-enough estimate for a spoken or
dashboard figure without needing a full internal-resistance load model.
"""


class BatteryETA:

    def __init__(self, idle_current_threshold_a: float, current_ema_alpha: float):
        self._idle_current_threshold_a = idle_current_threshold_a
        self._current_ema_alpha = current_ema_alpha
        self._smoothed_current_a = None

    @property
    def smoothed_current_a(self):
        return self._smoothed_current_a

    def update(self, current_a: float) -> None:
        """Fold one new current reading into the running EMA."""
        if self._smoothed_current_a is None:
            self._smoothed_current_a = current_a
        else:
            self._smoothed_current_a += (
                self._current_ema_alpha * (current_a - self._smoothed_current_a)
            )

    def estimate(self, charge_ah: float, design_capacity_ah: float):
        """Return (time_to_full_s, time_to_empty_s); the other is None in each case.

        Both are None while no reading has been smoothed yet, or the smoothed current
        is within the idle threshold - there is no meaningful rate to extrapolate from.
        """
        current = self._smoothed_current_a
        if current is None or abs(current) <= self._idle_current_threshold_a:
            return None, None

        if current > 0:
            remaining_ah = max(0.0, design_capacity_ah - charge_ah)
            return remaining_ah / current * 3600.0, None

        return None, charge_ah / abs(current) * 3600.0
