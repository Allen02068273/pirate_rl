from __future__ import annotations

import math
from enum import Enum, auto
from dataclasses import dataclass, field

class World:
    def __init__(self):
        self.time = 0.0
        self.entities: list[Entity] = []
        self.to_spawn: list[Entity] = []
        self.to_remove: set[Entity] = set()

    def spawn(self, entity: Entity):
        self.to_spawn.append(entity)

    def remove(self, entity: Entity):
        self.to_remove.add(entity)

    def step(self, dt: float):
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
        pass

@dataclass
class Entity:
    world: World
    transform: Transform = field(default_factory=Transform)

    def step(self, dt: float) -> None:
        pass

@dataclass
class ShipControls:
    throttle: float = 0.0
    steering: float = 0.0
    fire_left: bool = False
    fire_right: bool = False

class Ship(Entity):
    def __init__(self, world: World):
        super().__init__(world)
        self.ship_controls = ShipControls()
        self.length = 10.0
        self.width = 5.0
        self.linear_acceleration = 7.0
        self.angular_acceleration = 3.0
        self.velocity = Velocity()
        self.cannons: list[CannonGroup] = []

    def apply_controls(self, dt: float) -> None:
        forward = Vector2(1.0, 0.0).rotated(self.transform.angle)
        self.velocity.linear += (
            forward * self.ship_controls.throttle * self.linear_acceleration * dt
        )
        self.velocity.angular += (
            self.ship_controls.steering * self.angular_acceleration * dt
        )

    def apply_drag(self, dt: float) -> None:
        angular_drag = 0.8
        sideways_drag = 0.8
        forward_drag = 0.99

        self.velocity.angular *= math.exp(-angular_drag * dt)

        local_velocity = self.velocity.linear.rotated(-self.transform.angle)
        local_velocity.x *= math.exp(-forward_drag * dt)
        local_velocity.y *= math.exp(-sideways_drag * dt)
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
        if self.is_ready():
            for cannon in self.cannons:
                cannon.fire()

class Cannon:
    def __init__(self, local_transform: Transform, ship: Ship):
        self.local_transform = local_transform
        self.reload_complete_time = 0.0
        self.reload_time = 5.0
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
        cb = Cannonball(cannon_transform, 0.5, 5.0, self.ship)
        self.ship.world.spawn(cb)

class Cannonball(Entity):
    def __init__(
            self,
            transform: Transform,
            speed: float,
            lifetime: float,
            origin: Entity,
    ):
        super().__init__(origin.world, transform)
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

    def dot(self, other: Vector2) -> float:
        return self.x * other.x + self.y * other.y

    def cross(self, other: Vector2) -> float:
        return self.x * other.y - self.y * other.x

    def rotated(self, angle: float) -> Vector2:
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)

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

@dataclass
class Velocity:
    linear: Vector2 = field(default_factory=Vector2)
    angular: float = 0.0

class Renderer:
    def __init__(self):
        pass

    def render(self, world):
        pass