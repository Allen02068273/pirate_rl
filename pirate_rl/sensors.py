from collections.abc import Iterable
from dataclasses import dataclass, field
from math import inf, sqrt

from .collision import Capsule, Circle
from .geometry import Transform, Vector2, Velocity
from .simulation import Entity, Ship, World


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


def get_lidar_reading(base: Transform, range: float, entities: Iterable[Entity]):
    t_values = []
    for entity in entities:
        lidar_base = base.position
        lidar_vector = Vector2(x=range, y=0.0).rotated(base.angle)

        if isinstance(entity.rigid_body, Circle):
            circle_base = entity.transform.position
            circle_radius = entity.rigid_body.radius
            t_values.append(
                raycast_circle_check(
                    lidar_base, lidar_vector, circle_base, circle_radius
                )
            )

        if isinstance(entity.rigid_body, Capsule):
            capsule_vector = (
                Vector2(x=entity.rigid_body.half_length, y=0.0).rotated(
                    entity.transform.angle
                )
                * 2
            )
            capsule_base = entity.transform.position - capsule_vector / 2
            capsule_end = entity.transform.position + capsule_vector / 2
            capsule_radius = entity.rigid_body.radius

            t_values.append(
                raycast_circle_check(
                    lidar_base, lidar_vector, capsule_base, capsule_radius
                )
            )
            t_values.append(
                raycast_circle_check(
                    lidar_base, lidar_vector, capsule_end, capsule_radius
                )
            )
            t_values.append(
                raycast_rectangle_check(
                    lidar_base, lidar_vector, capsule_base, capsule_end, capsule_radius
                )
            )


def raycast_circle_check(
    ray_base: Vector2, ray_vector: Vector2, circle_base: Vector2, circle_radius: float
) -> float:
    a = ray_vector.magnitude_squared()
    b = 2 * ray_vector.dot(ray_base - circle_base)
    c = (ray_base - circle_base).magnitude_squared() - circle_radius * circle_radius

    sqrt_disc = sqrt(b * b - 4 * a * c)
    t_a = (-b + sqrt_disc) / (2 * a)
    t_b = (-b - sqrt_disc) / (2 * a)

    if 0.0 <= t_a <= 1.0 or 0.0 <= t_b <= 1.0:
        t = max(0.0, min(1.0, t_a, t_b))
        return t

    return inf


def raycast_rectangle_check(
    ray_base: Vector2,
    ray_vector: Vector2,
    capsule_base: Vector2,
    capsule_vector: Vector2,
    capsule_radius: float,
) -> float:
    # a = capsule_vector.cross()

    # return t

    return inf
