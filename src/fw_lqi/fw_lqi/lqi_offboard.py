# Copyright 2026 SirPicklee
# SPDX-License-Identifier: MIT
"""
LQI offboard flight controller node for the PX4 fixed-wing SITL.

PX4 flies the takeoff in mission mode. Once the aircraft is above
`engage_altitude`, this node flies the waypoint legs with L1 guidance and
the LQI controller, sending surface and throttle commands directly
(offboard, direct_actuator). After the last waypoint it sends
DO_LAND_START so PX4 resumes the mission at the landing sequence.

With `engage` false (default) the node only shadows the flight: it runs
guidance and control on the live state and logs the commands it would
send, but publishes nothing to PX4.
"""

import csv
import math

from fw_lqi.guidance import L1Guidance
from fw_lqi.lqi_core import LqiController, Measurement, Reference
from fw_lqi.px4_interface import (motor_controls, ned_to_body, quat_to_roll_pitch,
                                  servo_controls)
from px4_msgs.msg import (ActuatorMotors, ActuatorServos, AirspeedValidated,
                          OffboardControlMode, VehicleCommand, VehicleOdometry,
                          VehicleStatus)
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy)

TB2_LEGS = [1500.0, 3000.0, 120.0, 3000.0, 0.0, 120.0,
            1500.0, -3500.0, 100.0, 0.0, -3200.0, 80.0]
PX4_MODE_OFFBOARD = 6.0
COLUMNS = ['t', 'state', 'leg', 'cross_track', 'alt_ref', 'roll_ref',
           'elevator', 'aileron', 'rudder', 'throttle']


class LqiOffboard(Node):
    """Run L1 guidance and LQI control on PX4 odometry."""

    def __init__(self):
        super().__init__('lqi_controller')
        p = {
            'engage': False, 'engage_altitude': 55.0, 'airspeed_ref': 45.0,
            'trim_airspeed': 45.0, 'trim_alpha': 0.0, 'trim_elevator': 0.0,
            'trim_throttle': 0.6, 'k_lon': [0.0] * 12, 'k_lat': [0.0] * 10,
            'waypoints': TB2_LEGS, 'log_file': '/tmp/lqi_controller.csv',
        }
        for name, default in p.items():
            self.declare_parameter(name, default)
        g = {name: self.get_parameter(name).value for name in p}
        self.engage = g['engage']
        self.engage_alt = g['engage_altitude']
        self.v_ref = g['airspeed_ref']
        self.ctrl = LqiController(g['k_lon'], g['k_lat'], g['trim_airspeed'], g['trim_alpha'],
                                  g['trim_elevator'], g['trim_throttle'])
        wp = list(g['waypoints'])
        self.guide = L1Guidance([wp[i:i + 3] for i in range(0, len(wp), 3)])

        self.state = 'WAIT'
        self.nav_state, self.armed = -1, False
        self.airspeed = math.nan
        self.last_us = None
        self.t0_us = None
        self.file = open(g['log_file'], 'w', newline='')
        self.log = csv.writer(self.file)
        self.log.writerow(COLUMNS)

        qos_out = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pub_mode = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_out)
        self.pub_servos = self.create_publisher(ActuatorServos, '/fmu/in/actuator_servos', qos_out)
        self.pub_motors = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', qos_out)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_out)
        sd = qos_profile_sensor_data
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.on_status, sd)
        self.create_subscription(AirspeedValidated, '/fmu/out/airspeed_validated_v1',
                                 self.on_airspeed, sd)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.on_odom, sd)
        mode = 'ENGAGE' if self.engage else 'SHADOW (no commands sent)'
        self.get_logger().info(f'{mode}, {len(self.guide.waypoints)} waypoints, '
                               f'engage above {self.engage_alt:.0f} m')

    def on_status(self, msg):
        self.nav_state = msg.nav_state
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def on_airspeed(self, msg):
        self.airspeed = msg.true_airspeed_m_s

    def send_command(self, command, stamp, p1=0.0, p2=0.0):
        msg = VehicleCommand(timestamp=stamp, command=command, param1=p1, param2=p2,
                             target_system=1, target_component=1, source_system=1,
                             source_component=1, from_external=True)
        self.pub_cmd.publish(msg)

    def on_odom(self, msg):
        stamp = msg.timestamp                 # PX4 clock: timestamp sync is disabled
        dt = 0.0 if self.last_us is None else min(max((stamp - self.last_us) * 1e-6, 0.0), 0.1)
        self.last_us = stamp
        n, e, d = msg.position
        vn, ve, vd = msg.velocity
        altitude = -d
        if self.engage:
            self.pub_mode.publish(OffboardControlMode(timestamp=stamp, direct_actuator=True))

        if self.state == 'WAIT':
            mission = self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_MISSION
            if self.armed and mission and altitude > self.engage_alt:
                self.guide.reset(n, e)
                self.ctrl.reset()
                self.t0_us = stamp
                self.state = 'ACTIVE'
                if self.engage:
                    self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, stamp,
                                      1.0, PX4_MODE_OFFBOARD)
                self.get_logger().info(f'Taking over at {altitude:.1f} m')
            return
        if self.state != 'ACTIVE':
            return
        offboard = self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        if self.engage and not offboard and (stamp - self.t0_us) > 2e6:
            self.state = 'ABORTED'
            self.get_logger().warn(f'Left offboard (nav_state {self.nav_state}): stopping')
            return

        g = self.guide.update(n, e, vn, ve)
        if g.done:
            self.state = 'DONE'
            if self.engage:
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_LAND_START, stamp)
            self.get_logger().info('Last waypoint reached: handing over for landing')
            return
        roll, pitch = quat_to_roll_pitch(*msg.q)
        u, v, w = ned_to_body(msg.q, [vn, ve, vd])
        speed = self.airspeed if math.isfinite(self.airspeed) else math.sqrt(u * u + v * v + w * w)
        p, q, r = msg.angular_velocity
        meas = Measurement(u=u, v=v, w=w, p=p, q=q, r=r, phi=roll, theta=pitch,
                           altitude=altitude, airspeed=speed)
        cmd = self.ctrl.update(meas, Reference(g.altitude, self.v_ref, g.roll), dt)
        if self.engage and offboard:
            servos = servo_controls(cmd, self.ctrl.limits)
            self.pub_servos.publish(ActuatorServos(
                timestamp=stamp, timestamp_sample=stamp, control=servos))
            self.pub_motors.publish(ActuatorMotors(
                timestamp=stamp, timestamp_sample=stamp, control=motor_controls(cmd)))
        self.log.writerow([f'{(stamp - self.t0_us) * 1e-6:.3f}', self.state, g.leg,
                           f'{g.cross_track:.2f}', f'{g.altitude:.1f}',
                           f'{math.degrees(g.roll):.2f}', f'{math.degrees(cmd.elevator):.2f}',
                           f'{math.degrees(cmd.aileron):.2f}', f'{math.degrees(cmd.rudder):.2f}',
                           f'{cmd.throttle:.3f}'])


def main():
    rclpy.init()
    node = LqiOffboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.file.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
