from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from math import pi

from .geometry import Transform, Vector2, Velocity


@dataclass
class AABB:
    min: Vector2 = field(default_factory=Vector2)
    max: Vector2 = field(default_factory=Vector2)

    def contains(self, point: Vector2):
        return (
            self.min.x <= point.x <= self.max.x and self.min.y <= point.y <= self.max.y
        )

    def intersects(self, other: "AABB"):
        return not (
            self.max.x < other.min.x
            or self.min.x > other.max.x
            or self.max.y < other.min.y
            or self.min.y > other.max.y
        )


@dataclass
class RigidBody(ABC):
    @abstractmethod
    def sync_from_transform(
        self, transform: Transform, velocity: Velocity, dt: float
    ) -> None: ...

    @abstractmethod
    def sync_to_transform(
        self, transform: Transform, velocity: Velocity, dt: float
    ) -> None: ...

    @abstractmethod
    def get_aabb(self) -> AABB: ...


@dataclass
class Circle(RigidBody):
    radius: float
    is_static: bool = False
    world_point: Vector2 = field(default_factory=Vector2)
    prior_world_point: Vector2 = field(default_factory=Vector2)

    def __post_init__(self) -> None:
        if self.radius <= 0:
            raise ValueError("Circle radius must be positive")
        self.mass = pi * self.radius * self.radius

    def sync_from_transform(
        self, transform: Transform, velocity: Velocity, dt: float
    ) -> None:
        self.world_point = Vector2(transform.position.x, transform.position.y)
        if not self.is_static:
            self.prior_world_point = self.world_point - velocity.linear * dt

    def sync_to_transform(
        self, transform: Transform, velocity: Velocity, dt: float
    ) -> None:
        transform.position = Vector2(self.world_point.x, self.world_point.y)
        if not self.is_static:
            if self.radius <= 0:
                raise ValueError("dt must be positive")

            velocity.linear = (self.world_point - self.prior_world_point) / dt

    def get_aabb(self) -> AABB:
        return AABB(
            min=self.world_point - Vector2(self.radius, self.radius),
            max=self.world_point + Vector2(self.radius, self.radius),
        )


@dataclass
class Capsule(RigidBody):
    radius: float
    half_length: float
    is_static: bool = False
    world_point_a: Vector2 = field(default_factory=Vector2)
    world_point_b: Vector2 = field(default_factory=Vector2)
    prior_world_point_a: Vector2 = field(default_factory=Vector2)
    prior_world_point_b: Vector2 = field(default_factory=Vector2)

    def __post_init__(self) -> None:
        if self.radius <= 0:
            raise ValueError("Capsule radius must be positive")
        if self.half_length <= 0:
            raise ValueError("Capsule half_length must be positive")
        self.mass = self.half_length * self.radius * 2 + pi * self.radius * self.radius

    def a_as_circle(self) -> Circle:
        circle = Circle(
            radius=self.radius,
            is_static=self.is_static,
            world_point=self.world_point_a,
        )
        circle.mass = self.mass
        return circle

    def b_as_circle(self) -> Circle:
        circle = Circle(
            radius=self.radius,
            is_static=self.is_static,
            world_point=self.world_point_b,
        )
        circle.mass = self.mass
        return circle

    def sync_from_transform(
        self, transform: Transform, velocity: Velocity, dt: float
    ) -> None:
        self.world_point_a = transform.transform_point(Vector2(-self.half_length, 0.0))
        self.world_point_b = transform.transform_point(Vector2(self.half_length, 0.0))
        if not self.is_static:
            velocity_angular = (
                self.half_length
                * velocity.angular
                * Vector2(0.0, 1.0).rotated(transform.angle)
            )
            self.prior_world_point_a = (
                self.world_point_a - (velocity.linear - velocity_angular) * dt
            )
            self.prior_world_point_b = (
                self.world_point_b - (velocity.linear + velocity_angular) * dt
            )

    def sync_to_transform(
        self, transform: Transform, velocity: Velocity, dt: float
    ) -> None:
        transform.position = (self.world_point_a + self.world_point_b) / 2
        transform.angle = (self.world_point_b - self.world_point_a).angle()
        if not self.is_static:
            if self.radius <= 0:
                raise ValueError("dt must be positive")

            vel_a = (self.world_point_a - self.prior_world_point_a) / dt
            vel_b = (self.world_point_b - self.prior_world_point_b) / dt
            velocity.linear = (vel_a + vel_b) / 2.0
            velocity.angular = (vel_b - vel_a).dot(
                Vector2(0.0, 1.0).rotated(transform.angle)
            ) / (2.0 * self.half_length)

    def get_aabb(self) -> AABB:
        min_point = Vector2(
            min(self.world_point_a.x, self.world_point_b.x),
            min(self.world_point_a.y, self.world_point_b.y),
        )
        max_point = Vector2(
            max(self.world_point_a.x, self.world_point_b.x),
            max(self.world_point_a.y, self.world_point_b.y),
        )
        return AABB(
            min=min_point - Vector2(self.radius, self.radius),
            max=max_point + Vector2(self.radius, self.radius),
        )


class CollisionSystem:
    @staticmethod
    def resolve_circle_circle(circle_a: Circle, circle_b: Circle) -> bool:
        distance_vector = circle_b.world_point - circle_a.world_point
        radius = circle_a.radius + circle_b.radius
        if distance_vector.magnitude_squared() >= radius * radius:
            return False

        if distance_vector.magnitude_squared() > 0.0:
            penetration_vector = radius * distance_vector.norm() - distance_vector
        else:
            penetration_vector = radius * Vector2(1, 0)

        if not (circle_a.is_static or circle_b.is_static):
            circle_a.world_point -= (
                penetration_vector * circle_b.mass / (circle_a.mass + circle_b.mass)
            )
            circle_b.world_point += (
                penetration_vector * circle_a.mass / (circle_a.mass + circle_b.mass)
            )
        else:
            if not circle_a.is_static:
                circle_a.world_point -= penetration_vector
            if not circle_b.is_static:
                circle_b.world_point += penetration_vector
        return True

    @staticmethod
    def resolve_circle_capsule(circle: Circle, capsule: Capsule) -> bool:
        radius = circle.radius + capsule.radius
        capsule_point_vector = circle.world_point - capsule.world_point_a
        capsule_vector = capsule.world_point_b - capsule.world_point_a
        t = min(
            1,
            max(
                0,
                capsule_point_vector.dot(capsule_vector)
                / capsule_vector.magnitude_squared(),
            ),
        )
        distance_vector = capsule_point_vector - t * capsule_vector

        if distance_vector.magnitude_squared() >= radius * radius:
            return False

        if distance_vector.magnitude_squared() > 0.0:
            penetration_vector = radius * distance_vector.norm() - distance_vector
        else:
            penetration_vector = radius * Vector2(1, 0)

        if not (circle.is_static or capsule.is_static):
            circle.world_point += (
                penetration_vector * capsule.mass / (circle.mass + capsule.mass)
            )
            capsule.world_point_b -= (
                t * penetration_vector * circle.mass / (circle.mass + capsule.mass)
            )
            capsule.world_point_a -= (
                (1 - t)
                * penetration_vector
                * circle.mass
                / (circle.mass + capsule.mass)
            )
        else:
            if not circle.is_static:
                circle.world_point += penetration_vector
            if not capsule.is_static:
                capsule.world_point_b -= t * penetration_vector
                capsule.world_point_a -= (1 - t) * penetration_vector
        return True

    @staticmethod
    def resolve_capsule_capsule(capsule_a: Capsule, capsule_b: Capsule) -> bool:
        # Note: Since ships move slowly, endpoint-to-capsule detection is typically enough.
        circle = capsule_a.a_as_circle()
        if CollisionSystem.resolve_circle_capsule(circle, capsule_b):
            capsule_a.world_point_a = circle.world_point
            return True
        circle = capsule_a.b_as_circle()
        if CollisionSystem.resolve_circle_capsule(circle, capsule_b):
            capsule_a.world_point_b = circle.world_point
            return True
        circle = capsule_b.a_as_circle()
        if CollisionSystem.resolve_circle_capsule(circle, capsule_a):
            capsule_b.world_point_a = circle.world_point
            return True
        circle = capsule_b.b_as_circle()
        if CollisionSystem.resolve_circle_capsule(circle, capsule_a):
            capsule_b.world_point_b = circle.world_point
            return True
        return False

    @staticmethod
    def resolve_collision(body_a: RigidBody, body_b: RigidBody) -> bool:
        if not body_a.get_aabb().intersects(body_b.get_aabb()):
            return False

        if isinstance(body_a, Circle):
            if isinstance(body_b, Circle):
                return CollisionSystem.resolve_circle_circle(body_a, body_b)
            if isinstance(body_b, Capsule):
                return CollisionSystem.resolve_circle_capsule(body_a, body_b)
        elif isinstance(body_a, Capsule):
            if isinstance(body_b, Circle):
                return CollisionSystem.resolve_circle_capsule(body_b, body_a)
            if isinstance(body_b, Capsule):
                return CollisionSystem.resolve_capsule_capsule(body_a, body_b)

        raise TypeError(
            f"Unsupported collision types:"
            f"{type(body_a).__name__} and {type(body_b).__name__}"
        )
