from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum, auto

from .geometry import *


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
        pass

@dataclass
class Entity:
    world: World
    transform: Transform = field(default_factory=Transform)

    def step(self, dt: float) -> None:
        pass

@dataclass
class ShipControls:
    throttle: float = 0.0  # [-1, 1]
    steering: float = 0.0  # [-1, 1]
    fire_left: bool = False
    fire_right: bool = False

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

class Ship(Entity):
    def __init__(self, world: World, ship_config: ShipConfig):
        super().__init__(world)
        self.ship_controls = ShipControls()
        self.ship_config = ship_config
        self.velocity = Velocity()
        self.cannons: list[CannonGroup] = []

    def apply_controls(self, dt: float) -> None:
        forward = Vector2(1.0, 0.0).rotated(self.transform.angle)
        self.velocity.linear += (
            forward * self.ship_controls.throttle * self.ship_config.linear_acceleration * dt
        )
        self.velocity.angular += (
            self.ship_controls.steering * self.ship_config.angular_acceleration * dt
        )

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
