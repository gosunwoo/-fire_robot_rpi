import math

import pytest

from inno_autonav.mode6_smoke_assist import ObstacleHysteresis


def test_ultrasonic_hysteresis_stops_at_point_eight_and_releases_at_point_nine():
    gate = ObstacleHysteresis(0.8, 0.9)
    assert gate.update(1.0) is False
    assert gate.update(0.8) is True
    assert gate.update(0.85) is True
    assert gate.update(0.9) is False


def test_invalid_nan_reading_does_not_clear_an_existing_obstacle():
    gate = ObstacleHysteresis(0.8, 0.9)
    gate.update(0.5)
    assert gate.update(math.nan) is True


def test_invalid_hysteresis_distances_are_rejected():
    with pytest.raises(ValueError):
        ObstacleHysteresis(0.8, 0.7)
