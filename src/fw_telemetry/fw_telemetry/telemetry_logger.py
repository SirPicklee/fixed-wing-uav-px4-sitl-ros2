# Copyright 2026 SirPicklee
# SPDX-License-Identifier: MIT
"""
Log PX4 flight telemetry received over ROS 2 (uXRCE-DDS) to a CSV file.

Subscribes to:
  /fmu/out/vehicle_odometry       position, velocity, attitude, body rates
  /fmu/out/airspeed_validated_v1  true airspeed (message version 1)
  /fmu/out/vehicle_status_v1      navigation mode and arming state (version 1)

One CSV row is written per odometry message, decimated to `log_rate` Hz.
Time is taken from the PX4 message timestamp (synchronised by uXRCE-DDS),
relative to the first logged sample.
"""

import csv
import math

from fw_telemetry.conversions import quat_to_euler
from px4_msgs.msg import AirspeedValidated, VehicleOdometry, VehicleStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

COLUMNS = ['t', 'x', 'y', 'z', 'vx', 'vy', 'vz',
           'roll', 'pitch', 'yaw', 'p', 'q', 'r',
           'airspeed', 'nav_state', 'armed']


class TelemetryLogger(Node):
    """Write PX4 odometry, airspeed and status to a CSV file."""

    def __init__(self):
        super().__init__('telemetry_logger')
        self.declare_parameter('output_file', '/tmp/px4_telemetry.csv')
        self.declare_parameter('log_rate', 50.0)
        path = self.get_parameter('output_file').value
        self._min_dt_us = 1e6 / self.get_parameter('log_rate').value

        self._file = open(path, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(COLUMNS)

        self._t0_us = None
        self._last_us = None
        self._airspeed = math.nan
        self._nav_state = -1
        self._armed = 0
        self._rows = 0

        # PX4 publishes with best-effort reliability: subscribers must match.
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry',
                                 self._on_odometry, qos_profile_sensor_data)
        self.create_subscription(AirspeedValidated, '/fmu/out/airspeed_validated_v1',
                                 self._on_airspeed, qos_profile_sensor_data)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1',
                                 self._on_status, qos_profile_sensor_data)
        self.create_timer(5.0, self._report)
        self.get_logger().info(f'Logging to {path} at {1e6 / self._min_dt_us:.0f} Hz')

    def _on_airspeed(self, msg):
        self._airspeed = msg.true_airspeed_m_s

    def _on_status(self, msg):
        self._nav_state = msg.nav_state
        self._armed = int(msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def _on_odometry(self, msg):
        if self._last_us is not None and msg.timestamp - self._last_us < self._min_dt_us:
            return
        if self._t0_us is None:
            self._t0_us = msg.timestamp
        self._last_us = msg.timestamp

        roll, pitch, yaw = quat_to_euler(*msg.q)
        p, q, r = msg.angular_velocity
        self._writer.writerow([
            f'{(msg.timestamp - self._t0_us) * 1e-6:.3f}',
            *(f'{v:.3f}' for v in msg.position),
            *(f'{v:.3f}' for v in msg.velocity),
            *(f'{math.degrees(a):.3f}' for a in (roll, pitch, yaw, p, q, r)),
            f'{self._airspeed:.3f}', self._nav_state, self._armed,
        ])
        self._rows += 1

    def _report(self):
        self._file.flush()
        self.get_logger().info(f'{self._rows} rows logged')

    def destroy_node(self):
        """Flush and close the CSV file before shutting down."""
        self._file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
