from typing import Protocol

import pygame

from .simulation import ShipControls


class ShipController(Protocol):
    def get_controls(self, observation) -> ShipControls:
        ...

class KeyboardController:
    def get_controls(self, observation) -> ShipControls:
        keys = pygame.key.get_pressed()

        return ShipControls(
            throttle=float(keys[pygame.K_w]),
            steering=float(keys[pygame.K_d]) - float(keys[pygame.K_a]),
            fire_left=keys[pygame.K_LSHIFT],
            fire_right=keys[pygame.K_SPACE],
        )

class RLController:
    def __init__(self, policy):
        self.policy = policy

    def get_controls(self, observation) -> ShipControls:
        action = self.policy(observation)

        return ShipControls(
            throttle=action[0],
            steering=action[1],
            fire_left=action[2] > 0.5,
            fire_right=action[3] > 0.5,
        )
