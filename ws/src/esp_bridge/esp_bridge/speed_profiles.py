"""Authoritative requested velocity ceilings shared by controller and bridge."""

from dataclasses import dataclass


@dataclass(frozen=True)
class SpeedLimits:
    linear: float
    angular: float = 0.25


PROFILES = {'conservative': SpeedLimits(0.05), 'open_space': SpeedLimits(0.08)}


def speed_limits(profile):
    """Return ceilings in requested m/s and rad/s, not calibrated motion."""
    if profile not in PROFILES:
        raise ValueError(f'Unknown speed_profile {profile!r}; choose {tuple(PROFILES)}')
    return PROFILES[profile]


def declare_speed_profile(node):
    """Declare the same profile parameter in both ROS nodes."""
    node.declare_parameter('speed_profile', 'conservative')
    return speed_limits(node.get_parameter('speed_profile').value)
