"""Time-based display smoothing; UART values are never modified in place."""
import math

POSE_FIELDS = ('yaw_y_deg', 'pitch_x_deg', 'roll_z_deg', 'range_m',
               'optical_depth_m', 'distance_to_plane_m', 'tx_m')
OPTIONAL_FIELDS = ('along_track_m',)
ANGLE_FIELDS = frozenset(POSE_FIELDS[:3])


class PoseSmoother:
    def __init__(self, time_constant=.45):
        self.time_constant = time_constant
        self.reset()

    def reset(self):
        self.target = None
        self.current = None
        self.timestamp = None

    def set_target(self, pose, timestamp):
        if not pose or not pose.get('pose_valid') or any(
                pose.get(key) is None or not math.isfinite(pose[key]) for key in POSE_FIELDS):
            self.reset()
            return
        self.advance(timestamp)
        self.target = dict(pose)
        if self.current is None:
            self.current = dict(pose)
        self.timestamp = timestamp

    def advance(self, timestamp):
        if self.current is None or self.target is None:
            return None
        dt = max(0.0, timestamp-self.timestamp)
        alpha = -math.expm1(-dt/self.time_constant)
        filtered = dict(self.target)
        for key in POSE_FIELDS:
            previous = self.current[key]
            delta = self.target[key]-previous
            if key in ANGLE_FIELDS:
                delta = (delta+180) % 360-180
            value = previous+alpha*delta
            filtered[key] = (value+180) % 360-180 if key in ANGLE_FIELDS else value
        for key in OPTIONAL_FIELDS:
            previous, target = self.current.get(key), self.target.get(key)
            filtered[key] = None if previous is None or target is None else previous+alpha*(target-previous)
        self.timestamp = timestamp
        self.current = filtered
        return dict(filtered)
