import pygame

from .simulation import Cannonball, Ship, Vector2, World


class Renderer:
    def __init__(self):
        pygame.init()

        self.window = pygame.display.set_mode((400, 300))
        pygame.display.set_caption('Pirate RL')

    def render(self, world: World) -> None:
        self.window.fill((50, 130, 240))

        for entity in world.entities:
            if type(entity) is Ship:
                self.draw_ship(entity)
            elif type(entity) is Cannonball:
                self.draw_cannonball(entity)

        pygame.display.update()

    def draw_ship(self, ship: Ship):
        line_length = ship.ship_config.length - ship.ship_config.width
        line_width = int(ship.ship_config.width)

        base = ship.transform.position
        offset = Vector2(line_length / 2, 0.0).rotated(ship.transform.angle)
        side_offset = Vector2(0.0, line_width / 2).rotated(ship.transform.angle)

        # these are temporary for debugging
        base *= 5
        offset *= 5
        side_offset *= 5
        line_width *= 5

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

    def draw_cannonball(self, cannonball: Cannonball):
        point = cannonball.transform.position

        # this is temporary for debugging
        point *= 5

        pygame.draw.circle(
            self.window,
            (50, 50, 50),
            (point.x, point.y),
            3,
        )
