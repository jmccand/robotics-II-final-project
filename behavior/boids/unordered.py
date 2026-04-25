import numpy as np
from behavior.boids.boids import boids
from physics import dynamics, reflect, dt, BOUNDARY
from plot import setup_plot, draw, make_animation

cluster_size = 10

def simulate_random(frames=1000, save=None):
    np.random.seed(42)
    clusters = np.random.uniform(-BOUNDARY, BOUNDARY, (20, 2))
    states = np.vstack([
        np.hstack((
            cluster + np.random.uniform(-0.5, 0.5, (cluster_size, 2)),
            np.tile(np.random.uniform(-1, 1, (2,)), (cluster_size, 1))
        ))
        for cluster in clusters
    ])

    fig, ax = setup_plot(states)

    def frame(_):
        for i in range(states.shape[0]):
            action = boids(states[i], np.delete(states, i, axis=0))
            states[i] += dynamics(states[i], action) * dt
            states[i, 2:] = action
            reflect(states[i])
        draw(ax, states)

    make_animation(fig, frame, frames=frames, save=save)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--save', action='store_true')
    args = parser.parse_args()
    simulate_random(save='videos/unordered.mp4' if args.save else None)
