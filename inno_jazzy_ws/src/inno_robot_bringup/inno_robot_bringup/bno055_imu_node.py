"""BNO055 I2C driver for the MCU-055 breakout used by Mode 6."""

from __future__ import annotations

import math
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import String

try:
    import smbus
except ImportError:  # Hardware dependency is intentionally optional on dev PCs.
    smbus = None


CHIP_ID_REGISTER = 0x00
EXPECTED_CHIP_ID = 0xA0
PAGE_ID_REGISTER = 0x07
GYRO_DATA_REGISTER = 0x14
QUATERNION_DATA_REGISTER = 0x20
LINEAR_ACCELERATION_REGISTER = 0x28
MAGNETOMETER_DATA_REGISTER = 0x0E
CALIBRATION_STATUS_REGISTER = 0x35
UNIT_SELECTION_REGISTER = 0x3B
OPERATION_MODE_REGISTER = 0x3D
POWER_MODE_REGISTER = 0x3E
AXIS_MAP_CONFIG_REGISTER = 0x41
AXIS_MAP_SIGN_REGISTER = 0x42
CONFIG_MODE = 0x00
NDOF_MODE = 0x0C


def decode_i16_triplet(data, scale):
    """Decode three little-endian signed values and apply one scale."""
    if len(data) != 6 or not math.isfinite(float(scale)) or scale <= 0.0:
        raise ValueError('BNO055 vector data or scale is invalid')
    return tuple(value / float(scale) for value in struct.unpack('<hhh', bytes(data)))


def decode_quaternion(data):
    """Return ROS quaternion order x, y, z, w from BNO055 w, x, y, z."""
    if len(data) != 8:
        raise ValueError('BNO055 quaternion data must contain eight bytes')
    raw_w, raw_x, raw_y, raw_z = struct.unpack('<hhhh', bytes(data))
    scale = float(1 << 14)
    values = (
        raw_x / scale,
        raw_y / scale,
        raw_z / scale,
        raw_w / scale,
    )
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-6:
        raise ValueError('BNO055 returned a zero quaternion')
    return tuple(value / norm for value in values)


def decode_calibration(value):
    """Return system, gyro, accelerometer, magnetometer calibration levels."""
    value = int(value) & 0xFF
    return (
        (value >> 6) & 0x03,
        (value >> 4) & 0x03,
        (value >> 2) & 0x03,
        value & 0x03,
    )


class Bno055ImuNode(Node):
    def __init__(self):
        super().__init__('bno055_imu')
        defaults = {
            'i2c_bus': 1,
            'i2c_address': 0x28,
            'alternate_i2c_address': 0x29,
            'frame_id': 'imu_link',
            'publish_rate_hz': 20.0,
            'operation_mode': NDOF_MODE,
            'axis_map_config': 0x24,
            'axis_map_sign': 0x00,
            'reconnect_interval_sec': 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.bus_number = int(value('i2c_bus'))
        self.addresses = tuple(dict.fromkeys((
            int(value('i2c_address')),
            int(value('alternate_i2c_address')),
        )))
        self.frame_id = str(value('frame_id'))
        rate = float(value('publish_rate_hz'))
        self.operation_mode = int(value('operation_mode'))
        self.axis_map_config = int(value('axis_map_config'))
        self.axis_map_sign = int(value('axis_map_sign'))
        self.reconnect_interval = float(value('reconnect_interval_sec'))
        if (
            self.bus_number < 0
            or not self.frame_id
            or rate <= 0.0
            or self.reconnect_interval <= 0.0
            or self.operation_mode not in range(0x01, 0x0D)
            or any(address not in range(0x08, 0x78) for address in self.addresses)
        ):
            raise ValueError('BNO055 parameters are invalid')

        self.bus = None
        self.address = None
        self.last_connect_attempt = float('-inf')
        self.last_error = None
        self.last_calibration = None
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', 10)
        self.magnetic_publisher = self.create_publisher(
            MagneticField, '/imu/magnetic_field', 10
        )
        self.status_publisher = self.create_publisher(
            String, '/imu/status', 10
        )
        self.create_timer(1.0 / rate, self._on_timer)
        if smbus is None:
            self._status('OFFLINE:python3-smbus is not installed')
            self.get_logger().error(
                'BNO055 requires python3-smbus; run the dependency installer.'
            )
        else:
            self.get_logger().info(
                f'BNO055 waiting on I2C bus {self.bus_number}, '
                f'addresses={[hex(item) for item in self.addresses]}'
            )

    def _status(self, text):
        self.status_publisher.publish(String(data=str(text)))

    def _disconnect(self, error):
        message = str(error)
        if message != self.last_error:
            self.get_logger().error(f'BNO055 offline: {message}')
            self.last_error = message
        self._status(f'OFFLINE:{message}')
        if self.bus is not None:
            try:
                self.bus.close()
            except (AttributeError, OSError):
                pass
        self.bus = None
        self.address = None

    def _connect(self):
        if smbus is None:
            return False
        now = time.monotonic()
        if now - self.last_connect_attempt < self.reconnect_interval:
            return False
        self.last_connect_attempt = now
        try:
            bus = smbus.SMBus(self.bus_number)
            address = None
            for candidate in self.addresses:
                try:
                    if bus.read_byte_data(candidate, CHIP_ID_REGISTER) == EXPECTED_CHIP_ID:
                        address = candidate
                        break
                except OSError:
                    continue
            if address is None:
                bus.close()
                raise OSError(
                    'chip ID 0xA0 not found at '
                    + '/'.join(hex(item) for item in self.addresses)
                )
            self.bus = bus
            self.address = address
            self._write(OPERATION_MODE_REGISTER, CONFIG_MODE)
            time.sleep(0.03)
            self._write(PAGE_ID_REGISTER, 0x00)
            self._write(POWER_MODE_REGISTER, 0x00)
            self._write(UNIT_SELECTION_REGISTER, 0x00)
            self._write(AXIS_MAP_CONFIG_REGISTER, self.axis_map_config)
            self._write(AXIS_MAP_SIGN_REGISTER, self.axis_map_sign)
            self._write(OPERATION_MODE_REGISTER, self.operation_mode)
            time.sleep(0.03)
            self.last_error = None
            self.get_logger().info(
                f'BNO055 online: /dev/i2c-{self.bus_number} address={hex(address)}'
            )
            return True
        except (OSError, ValueError) as error:
            self._disconnect(error)
            return False

    def _write(self, register, value):
        self.bus.write_byte_data(self.address, int(register), int(value) & 0xFF)

    def _read(self, register, length):
        return self.bus.read_i2c_block_data(self.address, int(register), int(length))

    def _on_timer(self):
        if self.bus is None and not self._connect():
            return
        try:
            quaternion = decode_quaternion(self._read(QUATERNION_DATA_REGISTER, 8))
            gyro_dps = decode_i16_triplet(self._read(GYRO_DATA_REGISTER, 6), 16.0)
            linear_acceleration = decode_i16_triplet(
                self._read(LINEAR_ACCELERATION_REGISTER, 6), 100.0
            )
            magnetic_ut = decode_i16_triplet(
                self._read(MAGNETOMETER_DATA_REGISTER, 6), 16.0
            )
            calibration = decode_calibration(
                self.bus.read_byte_data(self.address, CALIBRATION_STATUS_REGISTER)
            )
        except (OSError, ValueError) as error:
            self._disconnect(error)
            return

        stamp = self.get_clock().now().to_msg()
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.frame_id
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = quaternion
        imu.angular_velocity.x = math.radians(gyro_dps[0])
        imu.angular_velocity.y = math.radians(gyro_dps[1])
        imu.angular_velocity.z = math.radians(gyro_dps[2])
        (
            imu.linear_acceleration.x,
            imu.linear_acceleration.y,
            imu.linear_acceleration.z,
        ) = linear_acceleration
        imu.orientation_covariance = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0076]
        imu.angular_velocity_covariance = [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.0004]
        imu.linear_acceleration_covariance = [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]
        self.imu_publisher.publish(imu)

        magnetic = MagneticField()
        magnetic.header = imu.header
        magnetic.magnetic_field.x = magnetic_ut[0] * 1e-6
        magnetic.magnetic_field.y = magnetic_ut[1] * 1e-6
        magnetic.magnetic_field.z = magnetic_ut[2] * 1e-6
        magnetic.magnetic_field_covariance = [1e-10, 0.0, 0.0, 0.0, 1e-10, 0.0, 0.0, 0.0, 1e-10]
        self.magnetic_publisher.publish(magnetic)

        if calibration != self.last_calibration:
            self.last_calibration = calibration
            system, gyro, accel, magnetometer = calibration
            status = (
                f'ONLINE:CALIB:SYS={system},GYRO={gyro},'
                f'ACCEL={accel},MAG={magnetometer}'
            )
            self._status(status)
            self.get_logger().info(f'BNO055 {status}')

    def destroy_node(self):
        if self.bus is not None:
            try:
                self.bus.close()
            except (AttributeError, OSError):
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Bno055ImuNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as error:
        print(f'bno055_imu: {error}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
