import numpy as np
from behavior.boids.boids import boids, neighbor_radius
from physics import dynamics, reflect, dt
from plot import setup_plot, draw, make_animation

def simulate_ordered(frames=100, save=None):
    states = np.array([[0.0, 0.0, 0.0, 0.5], [0.25, -0.25, 0.0, 1.0], [-0.25, -0.25, 0.0, 1.0]])
    fig, ax = setup_plot(states, focus=0)

    def frame(_):
        for i in range(states.shape[0]):
            action = boids(states[i], np.delete(states, i, axis=0))
            states[i] += dynamics(states[i], action) * dt
            states[i, 2:] = action
            reflect(states[i])
        draw(ax, states, focus=0, neighbor_radius=neighbor_radius)

    make_animation(fig, frame, frames=frames, save=save)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--save', action='store_true')
    args = parser.parse_args()
    simulate_ordered(save='videos/ordered.mp4' if args.save else None)
