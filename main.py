import pygame

from pirate_rl.controls import KeyboardController
from pirate_rl.rendering import Renderer
from pirate_rl.simulation import BRIG_CONFIG, World, spawn_ship


def main() -> None:
    world = World()
    ship = spawn_ship(world, BRIG_CONFIG)
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
