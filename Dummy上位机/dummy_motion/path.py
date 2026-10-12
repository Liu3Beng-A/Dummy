"""Straight translation and shortest-arc SO(3) interpolation share one progress."""
from .frames import interpolate


def smooth_progress(fraction):
    return fraction ** 3 * (10 + fraction * (-15 + 6 * fraction))


class LinePath:
    def __init__(self, start, target):
        self.start, self.target = start, target

    def at(self, fraction):
        return interpolate(self.start, self.target, fraction)
