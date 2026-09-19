import math

import pygame

from pirate_rl.controls import *
from pirate_rl.rendering import *
from pirate_rl.simulation import *

BRIG = ShipConfig(
    length=10.0,
    width=5.0,
    max_hull_health=100,
    linear_acceleration=4.0,
    angular_acceleration=1.0,
    forward_drag_rate=0.4,
    sideways_drag_rate=5.0,
    angular_drag_rate=2.0,
)


def new_brig(world: World) -> Ship:
    ship = Ship(world, BRIG)
    cannon_group_l = CannonGroup()
    cannon_group_r = CannonGroup()

    cannon_group_l.add_cannons([
        Transform(
            position=Vector2(3.0, -2.0),
            angle=-math.pi/2,
        ),
        Transform(
            position=Vector2(0.0, -2.0),
            angle=-math.pi/2,
        ),
        Transform(
            position=Vector2(-3.0, -2.0),
            angle=-math.pi/2,
        ),
    ], ship)
    cannon_group_r.add_cannons([
        Transform(
            position=Vector2(3.0, 2.0),
            angle=math.pi/2,
        ),
        Transform(
            position=Vector2(0.0, 2.0),
            angle=math.pi/2,
        ),
        Transform(
            position=Vector2(-3.0, 2.0),
            angle=math.pi/2,
        ),
    ], ship)

    ship.add_cannon_group(cannon_group_l)
    ship.add_cannon_group(cannon_group_r)

    return ship

def main() -> None:
    world = World()
    ship = new_brig(world)
    world.spawn(ship)
    controller = KeyboardController()
    renderer = Renderer()

    clock = pygame.time.Clock()
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        ship.set_controls(controller.get_controls(None))
        world.step(1/60)
        renderer.render(world)

        clock.tick(60)

    pygame.quit()

if __name__ == '__main__':
    main()