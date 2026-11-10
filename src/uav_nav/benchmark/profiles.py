"""Frozen evaluation profiles."""

from dataclasses import dataclass


@dataclass(frozen=True)
class Profile:
    timesteps: int
    checkpoints: tuple[int, ...]
    test_seed_start: int
    training_seeds: tuple[int, ...] = (0, 1, 2)
    test_layouts: int = 60


PROFILES = {
    "smoke": Profile(1024, (1024,), 20000, training_seeds=(0,), test_layouts=5),
    "exploratory": Profile(32768, (16384, 32768), 20000),
    "confirmation": Profile(300032, (100352, 200704, 300032), 30000),
}
