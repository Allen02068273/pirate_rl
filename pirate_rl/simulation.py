from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum, auto

from .geometry import Transform, Vector2, Velocity


@dataclass(frozen=True)
class ShipConfig:
    length: float
    width: float
    max_hull_health: float
    linear_acceleration: float
    angular_acceleration: float
    forward_drag_rate: float
    sideways_drag_rate: float
    angular_drag_rate: float
    cannon_group_l: tuple[Transform, ...]
    cannon_group_r: tuple[Transform, ...]

BRIG_CONFIG = ShipConfig(
    length=10.0,
    width=5.0,
    max_hull_health=100,
    linear_acceleration=4.0,
    angular_acceleration=1.0,
    forward_drag_rate=0.4,
    sideways_drag_rate=5.0,
    angular_drag_rate=2.0,
    cannon_group_l=(
        Transform(Vector2(3.0, -2.0), -math.pi / 2),
        Transform(Vector2(0.0, -2.0), -math.pi / 2),
        Transform(Vector2(-3.0, -2.0), -math.pi / 2),
    ),
    cannon_group_r=(
        Transform(Vector2(3.0, 2.0), math.pi / 2),
        Transform(Vector2(0.0, 2.0), math.pi / 2),
        Transform(Vector2(-3.0, 2.0), math.pi / 2),
    ),
)

def spawn_ship(world: World, config: ShipConfig) -> Ship:
    ship = Ship(world, config)
    cannon_group_l = CannonGroup()
    cannon_group_r = CannonGroup()

    cannon_group_l.add_cannons(config.cannon_group_l, ship)
    cannon_group_r.add_cannons(config.cannon_group_r, ship)

    ship.add_cannon_group(cannon_group_l)
    ship.add_cannon_group(cannon_group_r)

    world.spawn(ship)

    return ship

class World:
    def __init__(self):
        self.time = 0.0
        self.entities: list[Entity] = []
        self.to_spawn: list[Entity] = []
        self.to_remove: list[Entity] = []

    def spawn(self, entity: Entity):
        self.to_spawn.append(entity)

    def remove(self, entity: Entity):
        self.to_remove.append(entity)

    def step(self, dt: float):
        if dt <= 0:
            raise ValueError("dt must be positive")
        
        self.time += dt

        for entity in self.entities:
            entity.step(dt)

        self.handle_collisions()

        if self.to_remove:
            self.entities = [
                e for e in self.entities
                if e not in self.to_remove
            ]
            self.to_remove.clear()

        if self.to_spawn:
            self.entities.extend(self.to_spawn)
            self.to_spawn.clear()

    def handle_collisions(self):
        for entity in self.entities:
            entity.collider.sync_from_transform(entity.transform)

        ships, cannonballs = World.get_subtypes(self.entities)

        for i, ship_a in enumerate(ships):
            for ship_b in ships[i + 1:]:
                CollisionSystem.capsule_capsule(ship_a.collider, ship_b.collider)
            for cannonball in cannonballs:
                if (
                    not cannonball.origin is ship_a
                    and CollisionSystem.circle_capsule(cannonball.collider, ship_a.collider)
                ):
                        self.remove(cannonball)
            
        for entity in self.entities:
            entity.collider.sync_to_transform(entity.transform)

    @staticmethod
    def get_subtypes(entities: list[Entity]) -> tuple[list[Ship], list[Cannonball]]:
        ships, cannonballs = [], []
        for entity in entities:
            if isinstance(entity, Ship):
                ships.append(entity)
            if isinstance(entity, Cannonball):
                cannonballs.append(entity)

        return ships, cannonballs

@dataclass
class Circle:
    radius: float
    is_static: bool = False
    world_point: Vector2 = field(default_factory=Vector2)

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

    def a_as_circle(self):
        return Circle(
            radius=self.radius,
            is_static=self.is_static,
            world_point=self.world_point_a,
        )

    def b_as_circle(self):
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
    def entity_entity_collision(collider_a: Circle | Capsule, collider_b: Circle | Capsule) -> bool:
        func = None
        if isinstance(collider_a, Capsule) and isinstance(collider_b, Capsule):
            func = CollisionSystem.capsule_capsule
        elif isinstance(collider_a, Circle) and isinstance(collider_b, Circle):
            func = CollisionSystem.circle_circle
        elif isinstance(collider_a, Circle) and isinstance(collider_b, Capsule):
            func = CollisionSystem.circle_capsule
        elif isinstance(collider_a, Capsule) and isinstance(collider_b, Circle):
            func = CollisionSystem.capsule_circle

        if func:
            return func(collider_a, collider_b)

    @staticmethod
    def circle_circle(circle_a: Circle, circle_b: Circle) -> bool:
        distance_vector = circle_b.world_point - circle_a.world_point
        radius = circle_a.radius + circle_b.radius
        if distance_vector.magnitude_squared() >= radius * radius:
            return False
        
        if distance_vector.magnitude_squared > 0.0:
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
    def capsule_circle(capsule: Capsule, circle: Circle) -> bool:
        return CollisionSystem.circle_capsule(circle, capsule)

    @staticmethod
    def capsule_capsule(capsule_a: Capsule, capsule_b: Capsule) -> bool:
        return (
               CollisionSystem.circle_capsule(capsule_a.a_as_circle(), capsule_b)
            or CollisionSystem.circle_capsule(capsule_a.b_as_circle(), capsule_b)
            or CollisionSystem.circle_capsule(capsule_b.a_as_circle(), capsule_a)
            or CollisionSystem.circle_capsule(capsule_b.b_as_circle(), capsule_a)
        )

@dataclass
class Entity:
    world: World
    collider: Circle | Capsule
    transform: Transform = field(default_factory=Transform)

    def step(self, dt: float) -> None:
        pass

@dataclass
class ShipControls:
    throttle: float = 0.0  # [-1, 1]
    steering: float = 0.0  # [-1, 1]
    fire_left: bool = False
    fire_right: bool = False

class Ship(Entity):
    def __init__(self, world: World, ship_config: ShipConfig):
        super().__init__(world=world, collider=Capsule(
            radius=ship_config.width / 2,
            half_length=(ship_config.length - ship_config.width) / 2,
        ))
        self.ship_controls = ShipControls()
        self.ship_config = ship_config
        self.velocity = Velocity()
        self.cannons: list[CannonGroup] = []

    def set_controls(self, controls: ShipControls):
        self.ship_controls = controls

    def add_cannon_group(self, cannon_group: CannonGroup):
        self.cannons.append(cannon_group)

    def apply_controls(self, dt: float) -> None:
        forward = Vector2(1.0, 0.0).rotated(self.transform.angle)
        self.velocity.linear += (
            forward * self.ship_controls.throttle * self.ship_config.linear_acceleration * dt
        )
        self.velocity.angular += (
            self.ship_controls.steering * self.ship_config.angular_acceleration * dt
        )
        if self.ship_controls.fire_left:
            self.cannons[0].fire()
        if self.ship_controls.fire_right:
            self.cannons[1].fire()

    def apply_drag(self, dt: float) -> None:
        self.velocity.angular *= math.exp(-self.ship_config.angular_drag_rate * dt)

        local_velocity = self.velocity.linear.rotated(-self.transform.angle)
        local_velocity.x *= math.exp(-self.ship_config.forward_drag_rate * dt)
        local_velocity.y *= math.exp(-self.ship_config.sideways_drag_rate * dt)
        self.velocity.linear = local_velocity.rotated(self.transform.angle)

    def step(self, dt: float) -> None:
        self.apply_controls(dt)
        self.apply_drag(dt)

        self.transform.position += self.velocity.linear * dt
        self.transform.angle += self.velocity.angular * dt

class FireMode(Enum):
    VOLLEY = auto()
    FREE_FIRE = auto()

class CannonGroup:
    def __init__(self):
        self.cannons: list[Cannon] = []
        self.mode: FireMode = FireMode.VOLLEY

    def add_cannons(self, transforms: list[Transform], ship):
        for transform in transforms:
            self.cannons.append(Cannon(transform, ship))

    def get_reload_progress(self) -> float:
        if not self.cannons:
            return 0.0
        if self.mode is FireMode.VOLLEY:
            return min(cannon.get_reload_progress() for cannon in self.cannons)
        if self.mode is FireMode.FREE_FIRE:
            return max(cannon.get_reload_progress() for cannon in self.cannons)
        raise ValueError(f"Unknown fire mode: {self.mode}")

    def is_ready(self) -> bool:
        if not self.cannons:
            return False
        if self.mode is FireMode.VOLLEY:
            return all(cannon.is_ready() for cannon in self.cannons)
        if self.mode is FireMode.FREE_FIRE:
            return any(cannon.is_ready() for cannon in self.cannons)
        raise ValueError(f"Unknown fire mode: {self.mode}")

    def fire(self) -> None:
        if self.mode is FireMode.VOLLEY:
            if all(cannon.is_ready() for cannon in self.cannons):
                for cannon in self.cannons:
                    cannon.fire()

        elif self.mode is FireMode.FREE_FIRE:
            for cannon in self.cannons:
                cannon.fire()

class Cannon:
    def __init__(self, local_transform: Transform, ship: Ship):
        self.local_transform = local_transform
        self.reload_complete_time = 0.0
        self.reload_time = 1.0
        self.ship = ship

    def get_reload_progress(self) -> float:
        remaining = self.reload_complete_time - self.ship.world.time
        return max(0.0, min(1.0, 1.0 - remaining / self.reload_time))

    def is_ready(self) -> bool:
        return self.ship.world.time >= self.reload_complete_time

    def fire(self) -> None:
        if not self.is_ready():
            return
        self.reload_complete_time = self.ship.world.time + self.reload_time

        cannon_transform = self.ship.transform.transform(self.local_transform)
        cannonball = Cannonball(cannon_transform, 10.0, 1.0, self.ship)
        self.ship.world.spawn(cannonball)

class Cannonball(Entity):
    def __init__(
            self,
            transform: Transform,
            speed: float,
            lifetime: float,
            origin: Entity,
    ):
        super().__init__(world=origin.world, collider=Circle(radius=1.5), transform=transform)
        self.velocity = (
            origin.velocity.linear
            + Vector2(speed, 0.0).rotated(transform.angle)
        )
        self.origin = origin
        self.despawn_time = self.world.time + lifetime

    def step(self, dt: float) -> None:
        self.transform.position += self.velocity * dt
        if self.world.time >= self.despawn_time:
            self.world.remove(self)
