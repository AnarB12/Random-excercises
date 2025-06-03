import numpy as np
import pytest

from particle_simulation import Particle, PhysicsEnvironment


def test_particle_distance_to():
    p1 = Particle(position=[0, 0], velocity=[0, 0], mass=1.0, radius=0.1, color='b')
    p2 = Particle(position=[3, 4], velocity=[0, 0], mass=1.0, radius=0.1, color='r')
    assert pytest.approx(5.0) == p1.distance_to(p2)


def test_handle_wall_collisions_velocity_reversal():
    env = PhysicsEnvironment(width=10, height=10)
    particle = Particle(position=[-0.1, -0.1], velocity=[-1.0, -2.0], mass=1.0, radius=0.2, color='b')
    env.add_particle(particle)
    env.handle_wall_collisions()
    assert particle.position[0] == pytest.approx(particle.radius)
    assert particle.position[1] == pytest.approx(particle.radius)
    assert particle.velocity[0] == pytest.approx(1.0)
    assert particle.velocity[1] == pytest.approx(2.0)


def test_handle_particle_collisions_zero_distance():
    env = PhysicsEnvironment(width=10, height=10)
    p1 = Particle(position=[5, 5], velocity=[1, 0], mass=1.0, radius=0.5, color='b')
    p2 = Particle(position=[5, 5], velocity=[-1, 0], mass=1.0, radius=0.5, color='r')
    env.add_particle(p1)
    env.add_particle(p2)
    env.handle_particle_collisions()
    assert np.all(np.isfinite(p1.velocity))
    assert np.all(np.isfinite(p2.velocity))
