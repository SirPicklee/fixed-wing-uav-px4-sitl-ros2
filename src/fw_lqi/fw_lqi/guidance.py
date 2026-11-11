"""
L1 waypoint guidance for the LQI controller (numpy only, no ROS dependency).

Follows straight legs between waypoints in a local north-east frame and
returns a bank-angle and altitude reference. The lateral law is the
nonlinear L1 guidance of Park, Deyst and How (2004): with eta the angle
between the ground velocity and the line of sight to a reference point a
distance L1 ahead on the leg, the commanded lateral acceleration is
a = 2 V^2 sin(eta) / L1, flown as a coordinated bank phi = atan(a / g).
"""

from dataclasses import dataclass
import math

G0 = 9.80665


@dataclass
class GuidanceOutput:
    """References for the controller and leg status."""

    roll: float          # rad, positive right
    altitude: float      # m, positive up
    leg: int             # index of the active waypoint
    cross_track: float   # m, positive when right of the leg
    done: bool           # last waypoint reached


def wrap_pi(angle):
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class L1Guidance:
    """Leg-following L1 guidance over a list of (north, east, altitude)."""

    def __init__(self, waypoints, l1_distance=400.0, switch_distance=400.0,
                 max_bank=math.radians(30.0)):
        self.waypoints = [tuple(map(float, wp)) for wp in waypoints]
        if not self.waypoints:
            raise ValueError('at least one waypoint is required')
        self.l1 = l1_distance
        self.switch_distance = switch_distance
        self.max_bank = max_bank
        self.index = 0
        self.start = (0.0, 0.0)

    def reset(self, north, east):
        """Start the first leg at the current position."""
        self.index = 0
        self.start = (north, east)

    def _leg(self):
        if self.index == 0:
            a = self.start
        else:
            a = self.waypoints[self.index - 1][:2]
        return a, self.waypoints[self.index][:2]

    def update(self, north, east, v_north, v_east):
        """Return the references for the current position and ground velocity."""
        last = len(self.waypoints) - 1
        while True:
            (an, ae), (bn, be) = self._leg()
            length = math.hypot(bn - an, be - ae)
            un, ue = (bn - an) / length, (be - ae) / length
            along = (north - an) * un + (east - ae) * ue
            remaining = length - along
            if self.index < last and remaining <= self.switch_distance:
                self.index += 1
                continue
            break

        cross = (east - ae) * un - (north - an) * ue          # right of track positive
        done = self.index == last and remaining <= self.switch_distance
        # Reference point L1 ahead of the along-track projection (not past the leg end)
        ahead = min(max(along, 0.0) + self.l1, length)
        tn, te = an + un * ahead, ae + ue * ahead
        los = math.atan2(te - east, tn - north)
        speed = math.hypot(v_north, v_east)
        course = math.atan2(v_east, v_north)
        eta = wrap_pi(los - course)
        if abs(eta) > math.pi / 2:                            # target behind: turn hard
            eta = math.copysign(math.pi / 2, eta)
        distance = max(math.hypot(tn - north, te - east), 1.0)
        a_lat = 2.0 * speed**2 * math.sin(eta) / distance
        roll = max(-self.max_bank, min(self.max_bank, math.atan(a_lat / G0)))
        return GuidanceOutput(roll=roll, altitude=self.waypoints[self.index][2],
                              leg=self.index, cross_track=cross, done=done)
