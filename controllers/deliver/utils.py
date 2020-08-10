"""Utilities for the deliver controller."""
from math import acos, atan2, cos, sin, sqrt


class Vector:
    """Represents a vector in 2D cartesian space. Can also represent a point."""

    def __init__(self, x, y):
        """Set x and y."""
        self.x = x
        self.y = y

    def __repr__(self):
        """Return a printable Vector."""
        return f"({self.x}, {self.y})"

    def subtract(self, other):
        """Return the vector subtraction of self and another Vector."""
        x, y = (self.x - other.x), (self.y - other.y)
        return Vector(x, y)

    def mag(self):
        """Return the vector magnitude of self."""
        # a^2 = b^2 + c^2
        return sqrt(self.x ** 2 + self.y ** 2)

    def angle_from_x(self):
        """Return the angle of self from the x unit vector (counter-clockwise)."""
        return acos(self.x / self.mag())

    def angle(self):
        """Return the angle of self from the positive x axis (counter-clockwise)."""
        return atan2(self.y, self.x)


def normalise_angle(theta):
    """Ensure angle theta (radians) is in the range -pi to +pi."""
    return atan2(sin(theta), cos(theta))
