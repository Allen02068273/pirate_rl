from pirate_rl.controls import *
from pirate_rl.rendering import *
from pirate_rl.simulation import *

BRIG = ShipConfig(
    length=10.0,
    width=5.0,
    max_hull_health=100,
    linear_acceleration=7.0,
    angular_acceleration=3.0,
    forward_drag_rate=0.99,
    sideways_drag_rate=0.8,
    angular_drag_rate=0.8,
)

def main() -> None:
    world = World()
    ship = Ship(world, BRIG)
    controller = KeyboardController()
    renderer = Renderer()

    while True:
        ship.ship_controls = controller.get_controls()
        world.step(1/60)
        renderer.render(world)
