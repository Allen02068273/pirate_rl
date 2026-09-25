from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass, field
from enum import Enum, auto
from itertools import combinations

from .collision import Capsule, Circle, CollisionSystem
from .events import EventQueue
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

def spawn_ship(world: World, config: ShipConfig, transform: Transform) -> Ship:
    ship = Ship(world=world, ship_config=config, transform=transform)
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
        self.events = EventQueue()

    def spawn(self, entity: Entity) -> None:
        self.to_spawn.append(entity)

    def remove(self, entity: Entity) -> None:
        self.to_remove.append(entity)

    def schedule(self, time, callback) -> None:
        self.events.schedule(time, callback)

    def step(self, dt: float) -> None:
        if dt <= 0:
            raise ValueError("dt must be positive")
        
        self.time += dt

        for entity in self.entities:
            entity.step(dt)

        self.handle_collisions(dt)

        self.events.pop_ready(self.time)

        if self.to_remove:
            self.entities = [
                e for e in self.entities
                if e not in self.to_remove
            ]
            self.to_remove.clear()

        if self.to_spawn:
            self.entities.extend(self.to_spawn)
            self.to_spawn.clear()

    def handle_collisions(self, dt: float) -> None:
        for entity in self.entities:
            entity.collider.sync_from_transform(entity.transform, entity.velocity, dt)

        ships, cannonballs, islands = World.get_subtypes(self.entities)

        for ship_a, ship_b in combinations(ships, 2):
            CollisionSystem.resolve_capsule_capsule(ship_a.collider, ship_b.collider)

        for ship in ships:
            for cannonball in cannonballs:
                if (
                    not cannonball.origin is ship
                    and CollisionSystem.resolve_circle_capsule(cannonball.collider, ship.collider)
                ):
                    # self.remove(cannonball)
                    pass

        for ship in ships:
            for island in islands:
                CollisionSystem.resolve_circle_capsule(island.collider, ship.collider)
            
        for entity in self.entities:
            entity.collider.sync_to_transform(entity.transform, entity.velocity, dt)

    @staticmethod
    def get_subtypes(entities: list[Entity]) -> tuple[list[Ship], list[Cannonball]]:
        ships, cannonballs, islands = [], [], []
        for entity in entities:
            if isinstance(entity, Ship):
                ships.append(entity)
            elif isinstance(entity, Cannonball):
                cannonballs.append(entity)
            elif isinstance(entity, Island):
                islands.append(entity)

        return ships, cannonballs, islands

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

    def __post_init__(self):
        self.throttle = min(1, max(-1, self.throttle))
        self.steering = min(1, max(-1, self.steering))

class Ship(Entity):
    def __init__(self, world: World, ship_config: ShipConfig, transform: Transform):
        super().__init__(world=world, collider=Capsule(
            radius=ship_config.width / 2,
            half_length=(ship_config.length - ship_config.width) / 2,
        ), transform=transform)
        self.ship_controls = ShipControls()
        self.ship_config = ship_config
        self.velocity = Velocity()
        self.cannons: list[CannonGroup] = []

    def set_controls(self, controls: ShipControls) -> None:
        self.ship_controls = controls

    def add_cannon_group(self, cannon_group: CannonGroup) -> None:
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

    def add_cannons(self, transforms: Iterable[Transform], ship) -> None:
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
                    cannon.schedule_fire(0.1)

        elif self.mode is FireMode.FREE_FIRE:
            for cannon in self.cannons:
                cannon.schedule_fire(0.1)

class Cannon:
    def __init__(self, local_transform: Transform, ship: Ship):
        self.local_transform = local_transform
        self.ship = ship

        self.reload_complete_time = 0.0
        self.reload_time = 1.0

        self.fire_scheduled = False

    def get_reload_progress(self) -> float:
        remaining = self.reload_complete_time - self.ship.world.time
        return max(0.0, min(1.0, 1.0 - remaining / self.reload_time))

    def is_ready(self) -> bool:
        return self.ship.world.time >= self.reload_complete_time

    def schedule_fire(self, delay) -> None:
        if not self.is_ready() or self.fire_scheduled:
            return
        self.ship.world.schedule(
            time=self.ship.world.time + delay,
            callback=self.fire,
        )
        self.fire_scheduled = True

    def fire(self) -> None:
        if not self.is_ready():
            return

        cannon_transform = self.ship.transform.transform(self.local_transform)
        cannonball = Cannonball(
            transform=cannon_transform,
            speed=40.0,
            lifetime=1.0,
            origin=self.ship,
        )
        self.ship.world.spawn(cannonball)
        
        self.reload_complete_time = self.ship.world.time + self.reload_time
        self.fire_scheduled = False

class Cannonball(Entity):
    def __init__(
            self,
            transform: Transform,
            speed: float,
            lifetime: float,
            origin: Entity,
    ):
        super().__init__(world=origin.world, collider=Circle(radius=0.75), transform=transform)
        self.velocity = Velocity(
            linear=origin.velocity.linear
            + Vector2(speed, 0.0).rotated(transform.angle),
            angular=0.0,
        )
        self.origin = origin
        self.despawn_time = self.world.time + lifetime

    def step(self, dt: float) -> None:
        self.transform.position += self.velocity.linear * dt
        if self.world.time >= self.despawn_time:
            self.world.remove(self)

class Island(Entity):
    def __init__(self, world: World, transform: Transform):
        super().__init__(world=world, collider=Circle(radius=20, is_static=True), transform=transform)
        self.velocity = None
