from dataclasses import dataclass, field

from .geometry import Transform, Vector2


@dataclass
class Circle:
    radius: float
    is_static: bool = False
    world_point: Vector2 = field(default_factory=Vector2)

    def __post_init__(self) -> None:
        if self.radius <= 0:
            raise ValueError("Circle radius must be positive")

    def sync_from_transform(self, transform: Transform) -> None:
        self.world_point = Vector2(transform.position.x, transform.position.y)

    def sync_to_transform(self, transform: Transform) -> None:
        transform.position = Vector2(self.world_point.x, self.world_point.y)

@dataclass
class Capsule:
    radius: float
    half_length: float
    is_static: bool = False
    world_point_a: Vector2 = field(default_factory=Vector2)
    world_point_b: Vector2 = field(default_factory=Vector2)

    def __post_init__(self) -> None:
        if self.radius <= 0:
            raise ValueError("Capsule radius must be positive")
        if self.half_length <= 0:
            raise ValueError("Capsule half_length must be positive")

    def a_as_circle(self) -> Circle:
        return Circle(
            radius=self.radius,
            is_static=self.is_static,
            world_point=self.world_point_a,
        )

    def b_as_circle(self) -> Circle:
        return Circle(
            radius=self.radius,
            is_static=self.is_static,
            world_point=self.world_point_b,
        )

    def sync_from_transform(self, transform: Transform) -> None:
        self.world_point_a = transform.transform_point(Vector2(-self.half_length, 0.0))
        self.world_point_b = transform.transform_point(Vector2(self.half_length, 0.0))

    def sync_to_transform(self, transform: Transform) -> None:
        transform.position = (self.world_point_a + self. world_point_b) / 2
        transform.angle = (self.world_point_b - self.world_point_a).angle()

class CollisionSystem:
    @staticmethod
    def circle_circle(circle_a: Circle, circle_b: Circle) -> bool:
        distance_vector = circle_b.world_point - circle_a.world_point
        radius = circle_a.radius + circle_b.radius
        if distance_vector.magnitude_squared() >= radius * radius:
            return False
        
        if distance_vector.magnitude_squared() > 0.0:
            penetration_vector = radius * distance_vector.norm() - distance_vector
        else:
            penetration_vector = radius * Vector2(1, 0)

        if not (circle_a.is_static or circle_b.is_static):
            circle_a.world_point -= penetration_vector / 2
            circle_b.world_point += penetration_vector / 2
        else:
            if not circle_a.is_static:
                circle_a.world_point -= penetration_vector
            if not circle_b.is_static:
                circle_b.world_point += penetration_vector
        return True

    @staticmethod
    def circle_capsule(circle: Circle, capsule: Capsule) -> bool:
        radius = circle.radius + capsule.radius
        capsule_point_vector = (circle.world_point - capsule.world_point_a)
        capsule_vector = (capsule.world_point_b - capsule.world_point_a)
        t = min(1, max(0, capsule_point_vector.dot(capsule_vector) / capsule_vector.magnitude_squared()))
        distance_vector = capsule_point_vector - t * capsule_vector

        if distance_vector.magnitude_squared() >= radius * radius:
            return False

        if distance_vector.magnitude_squared() > 0.0:
            penetration_vector = radius * distance_vector.norm() - distance_vector
        else:
            penetration_vector = radius * Vector2(1, 0)

        if not (circle.is_static or capsule.is_static):
            circle.world_point += penetration_vector / 2
            capsule.world_point_b -= t * penetration_vector / 2
            capsule.world_point_a -= (1 - t) * penetration_vector / 2
        else:
            if not circle.is_static:
                circle.world_point += penetration_vector
            if not capsule.is_static:
                capsule.world_point_b -= t * penetration_vector
                capsule.world_point_a -= (1 - t) * penetration_vector
        return True

    @staticmethod
    def capsule_capsule(capsule_a: Capsule, capsule_b: Capsule) -> bool:
        # Note: Since ships move slowly, endpoint-to-capsule detection is typically enough.
        return (
               CollisionSystem.circle_capsule(capsule_a.a_as_circle(), capsule_b)
            or CollisionSystem.circle_capsule(capsule_a.b_as_circle(), capsule_b)
            or CollisionSystem.circle_capsule(capsule_b.a_as_circle(), capsule_a)
            or CollisionSystem.circle_capsule(capsule_b.b_as_circle(), capsule_a)
        )
