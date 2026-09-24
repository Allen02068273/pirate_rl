import pygame

from pirate_rl.controls import KeyboardController
from pirate_rl.geometry import Transform, Vector2
from pirate_rl.rendering import Camera, Renderer
from pirate_rl.simulation import BRIG_CONFIG, World, spawn_ship


def main() -> None:
    world = World()
    camera = Camera()
    controller = KeyboardController()
    renderer = Renderer()

    camera.scale = 5.0
    ship = spawn_ship(world, BRIG_CONFIG, Transform(position=Vector2(20, 15), angle=0))
    spawn_ship(world, BRIG_CONFIG, Transform(position=Vector2(40, 30), angle=-3.14/2))
    spawn_ship(world, BRIG_CONFIG, Transform())

    clock = pygame.time.Clock()
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        ship.set_controls(controller.get_controls(None))
        world.step(1/60)
        camera.frame_entities(world.entities, margin=10)
        # camera.transform.position = ship.transform.position
        # camera.transform.angle = ship.transform.angle + 3.14/2
        renderer.render(world, camera)

        clock.tick(60)

    pygame.quit()

if __name__ == '__main__':
    main()
