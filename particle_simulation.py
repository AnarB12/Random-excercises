import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Particle:
    def __init__(self, position, velocity, mass, radius, color):
        self.position = np.array(position, dtype='float64')
        self.velocity = np.array(velocity, dtype='float64')
        self.mass = mass
        self.radius = radius
        self.color = color

    def update_position(self, dt):
        self.position += self.velocity * dt

    def apply_force(self, force, dt):
        # Update velocity based on force and mass (F = ma -> a = F/m)
        acceleration = force / self.mass
        self.velocity += acceleration * dt

    def distance_to(self, other):
        return np.linalg.norm(self.position - other.position)

class PhysicsEnvironment:
    def __init__(self, width, height, gravity=-9.8):
        self.width = width
        self.height = height
        self.particles = []
        self.gravity = gravity
        self.time_step = 0.01

    def add_particle(self, particle):
        self.particles.append(particle)

    def calculate_gravitational_force(self, particle):
        force = np.array([0, particle.mass * self.gravity])
        return force

    def handle_collisions(self):
        # Handle collisions between particles
        for i in range(len(self.particles)):
            for j in range(i + 1, len(self.particles)):
                p1, p2 = self.particles[i], self.particles[j]
                distance = p1.distance_to(p2)
                min_distance = p1.radius + p2.radius

                if distance < min_distance:
                    # Simple elastic collision
                    normal = (p2.position - p1.position) / distance
                    relative_velocity = p2.velocity - p1.velocity
                    velocity_along_normal = np.dot(relative_velocity, normal)
                    if velocity_along_normal < 0:
                        restitution = 0.8  # Bounciness coefficient
                        impulse_magnitude = -(1 + restitution) * velocity_along_normal
                        impulse_magnitude /= (1 / p1.mass + 1 / p2.mass)

                        impulse = impulse_magnitude * normal
                        p1.velocity -= (1 / p1.mass) * impulse
                        p2.velocity += (1 / p2.mass) * impulse

    def update_particles(self):
        # Update forces and positions of particles
        for particle in self.particles:
            gravitational_force = self.calculate_gravitational_force(particle)
            particle.apply_force(gravitational_force, self.time_step)
            particle.update_position(self.time_step)

        # Collisions with walls
        for particle in self.particles:
            if particle.position[0] - particle.radius < 0 or particle.position[0] + particle.radius > self.width:
                particle.velocity[0] *= -1

            if particle.position[1] - particle.radius < 0 or particle.position[1] + particle.radius > self.height:
                particle.velocity[1] *= -1

        # Collisions between particles
        self.handle_collisions()

    def run_simulation(self, duration, visualize=True):
        if visualize:
            self.visualize_simulation(duration)
        else:
            for _ in range(int(duration / self.time_step)):
                self.update_particles()

    def visualize_simulation(self, duration):
        fig, ax = plt.subplots()
        ax.set_xlim(0, self.width)
        ax.set_ylim(0, self.height)
        ax.set_aspect('equal')
        ax.set_title('Particle Simulation')

        particles_patches = [plt.Circle(particle.position, particle.radius, 
                                        color=particle.color) for particle in self.particles]
        
        for patch in particles_patches:
            ax.add_patch(patch)

        def update(frame):
            self.update_particles()
            for i, particle in enumerate(self.particles):
                particles_patches[i].center = particle.position
            return particles_patches

        ani = animation.FuncAnimation(fig, update, frames=int(duration / self.time_step), 
                                      interval=self.time_step * 1000, blit=True)
        plt.show()



if __name__ == "__main__":

    environment = PhysicsEnvironment(width=10, height=10, gravity=-9.8)

    # Add particles to the environment
    particle_1 = Particle(position=[2, 5], velocity=[0.5, 1.0], mass=1.0, radius=0.2, color='blue')
    particle_2 = Particle(position=[5, 7], velocity=[-0.2, -0.5], mass=1.5, radius=0.3, color='red')
    particle_3 = Particle(position=[8, 4], velocity=[-0.3, 0.3], mass=1.0, radius=0.25, color='green')

    environment.add_particle(particle_1)
    environment.add_particle(particle_2)
    environment.add_particle(particle_3)

    environment.run_simulation(duration=20, visualize=True)