import math
import struct

import pytest

from inno_robot_bringup.bno055_imu_node import (
    decode_calibration,
    decode_i16_triplet,
    decode_quaternion,
)


def test_decode_i16_triplet_applies_scale():
    data = struct.pack('<hhh', 160, -320, 80)
    assert decode_i16_triplet(data, 16.0) == pytest.approx((10.0, -20.0, 5.0))


def test_decode_quaternion_reorders_wxyz_to_xyzw_and_normalizes():
    data = struct.pack('<hhhh', 1 << 14, 0, 0, 0)
    assert decode_quaternion(data) == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert math.isclose(sum(item * item for item in decode_quaternion(data)), 1.0)


def test_decode_calibration_fields():
    assert decode_calibration(0b11_10_01_00) == (3, 2, 1, 0)
