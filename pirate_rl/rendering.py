import pygame

from .geometry import Transform
from .simulation import Cannonball, Entity, Island, Ship, Vector2, World


class Camera:
    def __init__(self):
        self.transform = Transform()
        self.scale = 1.0

    def world_to_screen(self, transform: Transform) -> Transform:
        screen_width, screen_height = pygame.display.get_surface().get_size()
        center_offset = Vector2(screen_width, screen_height) / 2

        screen_transform = self.transform.inverse_transform(transform)
        screen_transform.position = screen_transform.position * self.scale + center_offset

        return screen_transform

    def screen_to_world(self, screen_point: Vector2) -> Vector2:
        screen_width, screen_height = pygame.display.get_surface().get_size()
        center_offset = Vector2(screen_width, screen_height) / 2

        world_point = self.transform.transform_point((screen_point - center_offset) / self.scale)

        return world_point

    def frame_entities(self, entities: list[Entity], margin: float = 0.0) -> None:
        min_x = min(e.transform.position.x for e in entities)
        max_x = max(e.transform.position.x for e in entities)
        min_y = min(e.transform.position.y for e in entities)
        max_y = max(e.transform.position.y for e in entities)

        screen_width, screen_height = pygame.display.get_surface().get_size()

        self.transform.position = Vector2(
            x=(min_x + max_x) / 2,
            y=(min_y + max_y) / 2,
        )
        self.scale = min(
            screen_width / (max_x - min_x + margin * 2),
            screen_height / (max_y - min_y + margin * 2),
        )

class Renderer:
    def __init__(self):
        pygame.init()

        self.window = pygame.display.set_mode((400, 300))
        pygame.display.set_caption('Pirate RL')

    def render(self, world: World, camera: Camera) -> None:
        self.window.fill((50, 130, 240))

        for entity in world.entities:
            if type(entity) is Ship:
                self.draw_ship(entity, camera)
            elif type(entity) is Cannonball:
                self.draw_cannonball(entity, camera)
            elif type(entity) is Island:
                self.draw_island(entity, camera)

        pygame.display.update()

    def draw_ship(self, ship: Ship, camera: Camera) -> None:
        transform = camera.world_to_screen(ship.transform)

        line_length = (ship.ship_config.length - ship.ship_config.width) * camera.scale
        line_width = int(ship.ship_config.width * camera.scale)

        base = transform.position
        offset = Vector2(line_length / 2, 0.0).rotated(transform.angle)
        side_offset = Vector2(0.0, line_width / 2).rotated(transform.angle)

        point_1 = base + offset
        point_2 = base - offset
        color = (100, 70, 45)

        pygame.draw.polygon(
            self.window,
            color,
            [
                [(point_1 + side_offset).x, (point_1 + side_offset).y],
                [(point_1 - side_offset).x, (point_1 - side_offset).y],
                [(point_2 - side_offset).x, (point_2 - side_offset).y],
                [(point_2 + side_offset).x, (point_2 + side_offset).y],
            ],
        )
        pygame.draw.circle(
            self.window,
            color,
            (point_1.x, point_1.y),
            line_width/2,
        )
        pygame.draw.circle(
            self.window,
            color,
            (point_2.x, point_2.y),
            line_width/2,
        )
        pygame.draw.circle(
            self.window,
            (150, 150, 150),
            (point_1.x, point_1.y),
            line_width/4,
        )

    def draw_cannonball(self, cannonball: Cannonball, camera: Camera) -> None:
        point = camera.world_to_screen(cannonball.transform).position
        radius = 0.75 * camera.scale

        pygame.draw.circle(
            self.window,
            (50, 50, 50),
            (point.x, point.y),
            radius,
        )

    def draw_island(self, island: Island, camera: Camera) -> None:
        point = camera.world_to_screen(island.transform).position
        radius = 20 * camera.scale

        pygame.draw.circle(
            self.window,
            (200, 140, 90),
            (point.x, point.y),
            radius,
        )
