import math
import sys
import unittest
from pathlib import Path

from builtin_interfaces.msg import Time

sys.path.append(str(Path(__file__).resolve().parents[1] / 'inno_drive_bridge'))

from cmdvel_to_esp32_serial import CmdVelToEsp32Serial  # noqa: E402


class DummyLogger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


class DummyPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class CmdVelToEsp32SerialParserTest(unittest.TestCase):
    def test_enc_abs_message_is_accepted_without_warning(self):
        node = CmdVelToEsp32Serial.__new__(CmdVelToEsp32Serial)
        node.logger = DummyLogger()
        node.ticks_publisher = DummyPublisher()
        node.status_publisher = DummyPublisher()
        node._publish_status = lambda text: None
        node.get_logger = lambda: node.logger

        node._parse_line('ENC_ABS,12345,10.0,20.0,0.5,1.0,0.02,0.04')

        self.assertEqual(node.logger.warnings, [])
        self.assertEqual(node.ticks_publisher.messages, [])

    def test_motor_targets_are_published_separately(self):
        node = CmdVelToEsp32Serial.__new__(CmdVelToEsp32Serial)
        node.left_motor_publisher = DummyPublisher()
        node.right_motor_publisher = DummyPublisher()

        node._publish_motor_targets(-123, 456)

        self.assertEqual(node.left_motor_publisher.messages[-1].data, -123)
        self.assertEqual(node.right_motor_publisher.messages[-1].data, 456)

    @staticmethod
    def _ultrasonic_parser_node():
        node = CmdVelToEsp32Serial.__new__(CmdVelToEsp32Serial)
        node.logger = DummyLogger()
        node.ultrasonic_publisher = DummyPublisher()
        node.ultrasonic_frame = 'ultrasonic_front'
        node.ultrasonic_field_of_view = 0.261799
        node.ultrasonic_min_range = 0.02
        node.ultrasonic_max_range = 4.0
        node.get_logger = lambda: node.logger
        node.get_clock = lambda: type('Clock', (), {
            'now': lambda _self: type('Now', (), {
                'to_msg': lambda _self: Time(),
            })(),
        })()
        return node

    def test_ultrasonic_telemetry_is_published_as_range(self):
        node = self._ultrasonic_parser_node()

        node._parse_line('US,12345,0.7500,1')

        message = node.ultrasonic_publisher.messages[-1]
        self.assertEqual(message.header.frame_id, 'ultrasonic_front')
        self.assertAlmostEqual(message.range, 0.75)

    def test_invalid_ultrasonic_telemetry_publishes_infinity(self):
        node = self._ultrasonic_parser_node()

        node._parse_line('US,12345,0.0000,0')

        self.assertTrue(math.isinf(node.ultrasonic_publisher.messages[-1].range))


if __name__ == '__main__':
    unittest.main()
