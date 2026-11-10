import math
from collections import deque


class AngleAverage:
    def __init__(self):
        self.reset()

    def reset(self):
        self.samples = deque()
        self.identity = None

    def update(self, identity, timestamp, values, window):
        if self.identity != identity or (self.samples and
                (timestamp < self.samples[-1][0] or timestamp - self.samples[-1][0] > max(1.0, window))):
            self.reset()
        self.identity = identity
        if not self.samples or timestamp != self.samples[-1][0]:
            self.samples.append((timestamp, dict(values)))
        while self.samples and self.samples[0][0] < timestamp - window:
            self.samples.popleft()
        result = {}
        for name in values:
            angles = [sample[name] for _, sample in self.samples]
            if name == 'front_angle_deg':
                result[name] = sum(angles) / len(angles)
            else:
                sine = sum(math.sin(math.radians(angle)) for angle in angles)
                cosine = sum(math.cos(math.radians(angle)) for angle in angles)
                result[name] = math.degrees(math.atan2(sine, cosine)) if math.hypot(sine, cosine) > 1e-8 else values[name]
        return result
