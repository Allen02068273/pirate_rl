from __future__ import annotations

from dataclasses import dataclass, field
from math import atan2, cos, sin, sqrt


@dataclass
class Vector2:
    x: float = 0.0
    y: float = 0.0

    def __add__(self, other: Vector2) -> Vector2:
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: Vector2) -> Vector2:
        return Vector2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> Vector2:
        return Vector2(self.x * scalar, self.y * scalar)

    def __rmul__(self, scalar: float) -> Vector2:
        return self * scalar

    def __truediv__(self, scalar: float) -> Vector2:
        return Vector2(self.x / scalar, self.y / scalar)

    def __rtruediv__(self, scalar: float) -> Vector2:
        return Vector2(scalar / self.x, scalar / self.y)

    def dot(self, other: Vector2) -> float:
        return self.x * other.x + self.y * other.y

    def cross(self, other: Vector2) -> float:
        return self.x * other.y - self.y * other.x

    def magnitude_squared(self) -> float:
        return self.x * self.x + self.y * self.y

    def norm(self) -> float:
        return self / sqrt(self.magnitude_squared())

    def angle(self) -> float:
        return atan2(self.y, self.x)

    def rotated(self, angle: float) -> Vector2:
        cos_a = cos(angle)
        sin_a = sin(angle)

        return Vector2(
            cos_a * self.x - sin_a * self.y,
            sin_a * self.x + cos_a * self.y,
        )

@dataclass
class Transform:
    position: Vector2 = field(default_factory=Vector2)
    angle: float = 0.0

    def transform_point(self, local_point: Vector2) -> Vector2:
        return self.position + local_point.rotated(self.angle)

    def inverse_transform_point(self, world_point: Vector2) -> Vector2:
        return (world_point - self.position).rotated(-self.angle)

    def transform(self, local: Transform) -> Transform:
        return Transform(
            position=self.transform_point(local.position),
            angle=self.angle + local.angle,
        )

    def inverse_transform(self, local: Transform) -> Transform:
        return Transform(
            position=self.inverse_transform_point(local.position),
            angle=local.angle - self.angle,
        )

@dataclass
class Velocity:
    linear: Vector2 = field(default_factory=Vector2)
    angular: float = 0.0
