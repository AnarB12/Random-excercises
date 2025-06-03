import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from typing import List, Tuple

plt.rcParams['font.family'] = 'Times New Roman'


class Particle:
    """
    A class representing a 2D particle.

    Attributes
    ----------
    position : np.ndarray
        The current position of the particle in 2D space.
        Position at time step t: x(t) = x(t0) + v(t0) * (t - t0)

    velocity : np.ndarray
        The current velocity of the particle in 2D space.
        Velocity update when a force F is applied:
        v(t + Δt) = v(t) + (F / m) * Δt

    mass : float
        The mass of the particle.

    radius : float
        The radius of the particle.

    color : str
        A color string recognized by matplotlib.
    """

    def __init__(self,
                 position: Tuple[float, float],
                 velocity: Tuple[float, float],
                 mass: float,
                 radius: float,
                 color: str):
        self.position = np.array(position, dtype='float64')
        self.velocity = np.array(velocity, dtype='float64')
        self.mass = mass
        self.radius = radius
        self.color = color

    def update_position(self, dt: float) -> None:
        """
        Update the particle's position based on its velocity and the given time step.

        Position update:
        x(t + Δt) = x(t) + v(t) * Δt

        Parameters
        ----------
        dt : float
            The time step for updating the position.
        """
        self.position += self.velocity * dt

    def apply_force(self, force: np.ndarray, dt: float) -> None:
        """
        Apply a given force to the particle, updating its velocity.

        From Newton's second law (F = m * a), we have a = F / m.
        Velocity update:
        v(t + Δt) = v(t) + (F / m) * Δt

        Parameters
        ----------
        force : np.ndarray
            The force vector applied to the particle.
        dt : float
            The time step over which the force is applied.
        """
        acceleration = force / self.mass
        self.velocity += acceleration * dt

    def distance_to(self, other: 'Particle') -> float:
        """
        Compute the distance from this particle to another particle.

        Distance d between two particles at positions x1 and x2:
        d = |x2 - x1|

        Parameters
        ----------
        other : Particle
            Another particle object.

        Returns
        -------
        float
            The Euclidean distance between the two particles.
        """
        return np.linalg.norm(self.position - other.position)


class PhysicsEnvironment:
    """
    A class representing a 2D physics environment with gravitational force, 
    multiple particles, and collision handling.

    Attributes
    ----------
    width : float
        The width of the environment.
    height : float
        The height of the environment.

    gravity : float
        The gravitational acceleration (negative value indicates downward).
        Gravitational force on a particle: F_g = m * g

    particles : List[Particle]
        A list of Particle objects present in the environment.

    time_step : float
        The time step used for numerical simulation.
    """

    def __init__(self, width: float, height: float, gravity: float = -9.8, time_step: float = 0.01):
        """
        Initialize the physics environment.

        Parameters
        ----------
        width : float
            The width of the simulation area.
        height : float
            The height of the simulation area.
        gravity : float, optional
            The gravitational acceleration (default is -9.8).
        time_step : float, optional
            The fixed time step for the simulation updates (default is 0.01).
        """
        self.width = width
        self.height = height
        self.gravity = gravity
        self.time_step = time_step
        self.particles: List[Particle] = []

    def add_particle(self, particle: Particle) -> None:
        """
        Add a particle to the environment.

        Parameters
        ----------
        particle : Particle
            The particle to add to the environment.
        """
        self.particles.append(particle)

    def calculate_gravitational_force(self, particle: Particle) -> np.ndarray:
        """
        Calculate the gravitational force on a particle.

        Gravitational force:
        F_g = m * g (where g is the gravitational acceleration vector)

        Parameters
        ----------
        particle : Particle
            The particle on which gravitational force acts.

        Returns
        -------
        np.ndarray
            The gravitational force vector acting on the particle.
        """
        return np.array([0.0, particle.mass * self.gravity])

    def handle_particle_collisions(self) -> None:
        """
        Handle elastic collisions between particles in the environment.

        If two particles collide, their velocities are adjusted to reflect 
        an elastic collision. The impulse J is computed based on the relative 
        velocity along the line of impact and the masses of the two particles.

        In a simple elastic collision along the normal direction n:
        J = -(1 + restitution) * (relative_velocity · n) / ( (1/m1) + (1/m2) )
        Then each particle's velocity is updated based on J and its mass.
        """
        n = len(self.particles)
        for i in range(n):
            for j in range(i + 1, n):
                p1, p2 = self.particles[i], self.particles[j]
                distance = p1.distance_to(p2)
                min_distance = p1.radius + p2.radius

                # If particles overlap, handle collision
                if distance < min_distance:
                    # Avoid division by zero if particles occupy the same point
                    if distance == 0:
                        # Arbitrary normal to separate completely overlapping particles
                        normal = np.array([1.0, 0.0])
                        distance = min_distance
                    else:
                        # Compute the normal vector from p1 to p2
                        normal = (p2.position - p1.position) / distance
                    relative_velocity = p2.velocity - p1.velocity
                    velocity_along_normal = np.dot(relative_velocity, normal)

                    # Only resolve if particles are moving closer
                    if velocity_along_normal < 0:
                        restitution = 0.8  # A measure of "bounciness"
                        impulse_magnitude = -(1 + restitution) * velocity_along_normal
                        impulse_magnitude /= (1 / p1.mass + 1 / p2.mass)

                        impulse = impulse_magnitude * normal
                        p1.velocity -= (1 / p1.mass) * impulse
                        p2.velocity += (1 / p2.mass) * impulse

    def handle_wall_collisions(self) -> None:
        """
        Handle collisions of particles with the walls of the environment.

        If a particle hits a wall, the corresponding velocity component is reversed, 
        simulating an elastic collision with the wall.
        """
        for particle in self.particles:
            # Check left/right walls
            if particle.position[0] - particle.radius < 0:
                particle.position[0] = particle.radius
                particle.velocity[0] *= -1
            elif particle.position[0] + particle.radius > self.width:
                particle.position[0] = self.width - particle.radius
                particle.velocity[0] *= -1

            # Check bottom/top walls
            if particle.position[1] - particle.radius < 0:
                particle.position[1] = particle.radius
                particle.velocity[1] *= -1
            elif particle.position[1] + particle.radius > self.height:
                particle.position[1] = self.height - particle.radius
                particle.velocity[1] *= -1

    def update_particles(self) -> None:
        """
        Update all particles for one simulation step.

        Steps for each particle:
        1. Compute gravitational force and update velocity:
           v(t + Δt) = v(t) + (F_g / m) * Δt
        2. Update position:
           x(t + Δt) = x(t) + v(t) * Δt

        After updating all particles, handle collisions with walls and other particles.
        """
        for particle in self.particles:
            gravitational_force = self.calculate_gravitational_force(particle)
            particle.apply_force(gravitational_force, self.time_step)
            particle.update_position(self.time_step)

        # Handle wall collisions
        self.handle_wall_collisions()

        # Handle particle collisions
        self.handle_particle_collisions()

    def run_simulation(self, duration: float, visualize: bool = True) -> None:
        """
        Run the simulation for a given duration.

        Parameters
        ----------
        duration : float
            The total simulation time in seconds.
        visualize : bool
            If True, displays an animated visualization of the simulation.
            Otherwise, runs the simulation without visualization.
        """
        steps = int(duration / self.time_step)
        if steps < 1:
            raise ValueError("Simulation duration is too short relative to the time step.")

        if visualize:
            self.visualize_simulation(duration)
        else:
            for _ in range(steps):
                self.update_particles()

    def visualize_simulation(self, duration: float) -> None:
        """
        Visualize the simulation using matplotlib animation.

        Parameters
        ----------
        duration : float
            The total simulation time in seconds.
        """
        steps = int(duration / self.time_step)
        fig, ax = plt.subplots()
        ax.set_xlim(0, self.width)
        ax.set_ylim(0, self.height)
        ax.set_aspect('equal', 'box')
        ax.set_title('Particle Simulation', fontname='Times New Roman')
        ax.set_xlabel('X Position', fontname='Times New Roman')
        ax.set_ylabel('Y Position', fontname='Times New Roman')

        # Create a circle patch for each particle
        particle_patches = [
            plt.Circle(p.position, p.radius, color=p.color) for p in self.particles
        ]
        for patch in particle_patches:
            ax.add_patch(patch)

        def update(frame):
            self.update_particles()
            for i, p in enumerate(self.particles):
                particle_patches[i].center = p.position
            return particle_patches

        ani = animation.FuncAnimation(
            fig,
            update,
            frames=steps,
            interval=self.time_step * 1000,
            blit=True
        )
        plt.show()


if __name__ == "__main__":
    env = PhysicsEnvironment(width=10.0, height=10.0, gravity=-9.8, time_step=0.01)

    # Create some particles
    particle_1 = Particle(position=[2, 5], velocity=[0.5, 1.0], mass=1.0, radius=0.2, color='blue')
    particle_2 = Particle(position=[5, 7], velocity=[-0.2, -0.5], mass=1.5, radius=0.3, color='red')
    particle_3 = Particle(position=[8, 4], velocity=[-0.3, 0.3], mass=1.0, radius=0.25, color='green')
    particle_4 = Particle(position=[1, 1], velocity=[0.8, 0.5], mass=0.8, radius=0.2, color='cyan')
    particle_5 = Particle(position=[9, 8], velocity=[-0.4, -0.1], mass=1.2, radius=0.15, color='magenta')
    particle_6 = Particle(position=[4, 4], velocity=[0.0, -0.5], mass=1.1, radius=0.25, color='yellow')
    particle_7 = Particle(position=[3, 9], velocity=[0.2, -0.3], mass=0.9, radius=0.2, color='orange')
    particle_8 = Particle(position=[6, 2], velocity=[-0.6, 0.4], mass=1.3, radius=0.2, color='purple')
    particle_9 = Particle(position=[7, 5], velocity=[0.4, -0.2], mass=1.5, radius=0.3, color='brown')
    particle_10 = Particle(position=[2, 8], velocity=[-0.1, -0.4], mass=1.0, radius=0.25, color='gray')

    # Add particles to the environment
    env.add_particle(particle_1)
    env.add_particle(particle_2)
    env.add_particle(particle_3)
    env.add_particle(particle_4)
    env.add_particle(particle_5)
    env.add_particle(particle_6)
    env.add_particle(particle_7)
    env.add_particle(particle_8)
    env.add_particle(particle_9)
    env.add_particle(particle_10)

    env.run_simulation(duration=10, visualize=True)

