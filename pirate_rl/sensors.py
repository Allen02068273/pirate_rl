from collections.abc import Iterable
from dataclasses import dataclass, field
from math import inf, pi, sqrt

from .collision import Capsule, Circle
from .geometry import Transform, Vector2, Velocity
from .simulation import Entity, Ship, World

# Floating-point tolerance for geometric zero comparisons
EPS = 1e-9


@dataclass
class Observation:
    velocity: Velocity = field(default_factory=Velocity)
    lower_radial_scan: list[float] = field(default_factory=list)  # islands
    upper_radial_scan: list[tuple[float, float]] = field(default_factory=list)  # ships


def get_observation(world: World, ship: Ship) -> Observation:
    observation = Observation()
    observation.velocity = ship.velocity
    # observation.lower_radial_scan = get_lidat_reading()

    return observation


@dataclass
class Sensor:
    local_transform: Transform
    entity: Entity

    def observe(self, world: World) -> list[float]:
        return []


class Lidar(Sensor):
    def __init__(self, entity: Entity):
        super().__init__(local_transform=Transform(), entity=entity)
        self.max_range: float = 100.0
        self.rays: int = 32

    def observe(self, world: World) -> list[float]:
        range_values = []
        entities = [e for e in world.entities if e is not self.entity]
        ray_transform = self.entity.transform.transform(self.local_transform)

        for _ in range(self.rays):
            distance, _entity = raycast_entities(
                ray_transform, self.max_range, entities
            )
            t = distance / self.max_range
            range_values.append(t)

            ray_transform.angle += 2.0 * pi / self.rays

        return range_values


def raycast_entities(
    ray_transform: Transform, max_range: float, entities: Iterable[Entity]
) -> tuple[float, Entity | None]:
    nearest_distance = max_range
    nearest_entity = None

    ray_origin = ray_transform.position
    ray_unit_vector = Vector2(x=1.0, y=0.0).rotated(ray_transform.angle)
    for entity in entities:
        if isinstance(entity.rigid_body, Circle):
            circle_position = entity.transform.position
            circle_radius = entity.rigid_body.radius
            distance = raycast_circle(
                ray_origin, ray_unit_vector, circle_position, circle_radius
            )

        elif isinstance(entity.rigid_body, Capsule):
            capsule = entity.rigid_body
            bounding_radius = capsule.radius + capsule.half_length

            t = raycast_circle(
                ray_origin,
                ray_unit_vector,
                entity.transform.position,
                bounding_radius,
            )
            if t >= nearest_distance:
                continue

            axis = Vector2(x=1.0, y=0.0).rotated(entity.transform.angle)
            capsule_base = (
                entity.transform.position - axis * entity.rigid_body.half_length
            )
            capsule_end = (
                entity.transform.position + axis * entity.rigid_body.half_length
            )
            capsule_radius = entity.rigid_body.radius

            distance = min(
                nearest_distance,
                raycast_circle(
                    ray_origin, ray_unit_vector, capsule_base, capsule_radius
                ),
                raycast_circle(
                    ray_origin, ray_unit_vector, capsule_end, capsule_radius
                ),
                raycast_capsule_sides(
                    ray_origin,
                    ray_unit_vector,
                    capsule_base,
                    capsule_end - capsule_base,
                    capsule_radius,
                ),
            )
        else:
            raise TypeError(
                f"Unsupported raycast rigid body type: "
                f"{type(entity.rigid_body).__name__}"
            )

        if distance < nearest_distance:
            nearest_distance = distance
            nearest_entity = entity

    return (nearest_distance, nearest_entity)


def raycast_circle(
    ray_origin: Vector2,
    ray_unit_vector: Vector2,
    circle_base: Vector2,
    circle_radius: float,
) -> float:
    # solve via quadratic formula (a=1.0)
    b = 2.0 * ray_unit_vector.dot(ray_origin - circle_base)
    c = (ray_origin - circle_base).magnitude_squared() - circle_radius * circle_radius

    disc = b * b - 4 * c
    if disc < 0.0:
        return inf

    sqrt_disc = sqrt(disc)
    t_a = (-b + sqrt_disc) / 2.0
    t_b = (-b - sqrt_disc) / 2.0

    candidate_t = [t for t in (t_a, t_b) if t >= 0.0]

    return min(candidate_t) if candidate_t else inf


def raycast_capsule_sides(
    ray_origin: Vector2,
    ray_unit_vector: Vector2,
    capsule_base: Vector2,
    capsule_vector: Vector2,
    capsule_radius: float,
) -> float:
    capsule_len_squared = capsule_vector.magnitude_squared()
    if capsule_len_squared < EPS:
        # no sides on a degenerate capsule
        return inf

    capsule_vector_normalized = capsule_vector.norm()
    a = capsule_vector_normalized.cross(ray_origin - capsule_base)
    b = capsule_vector_normalized.cross(ray_unit_vector)

    if abs(b) < EPS:
        # if the capsule is parallel with the ray,
        # any intersection will be caught by the circle endpoints
        return inf

    t_a = (capsule_radius - a) / b
    t_b = (-capsule_radius - a) / b

    candidate_t = [
        t
        for t in (t_a, t_b)
        if t >= 0.0
        and 0.0
        <= capsule_vector.dot(ray_origin + t * ray_unit_vector - capsule_base)
        / capsule_len_squared
        <= 1.0
    ]

    return min(candidate_t) if candidate_t else inf
