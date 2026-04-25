import numpy as np
from physics import dynamics, reflect, dt, BOUNDARY

w_sep = 0.3
w_align = 1.5
w_coh = 1.0
u_scale = 0.15
neighbor_radius = 1
num_neighbors = 5

# state: [x, y, v_x, v_y]

def boids(state, neighbor_states):
    closest = np.argsort(np.linalg.norm(neighbor_states[:, :2] - state[:2], axis=1))[:num_neighbors]
    neighbors = neighbor_states[closest]
    if neighbors.shape[0] == 0:
        return np.zeros(2)
    position = state[:2]
    velocity = state[2:]
    neighbor_positions = neighbors[:, :2]
    neighbor_velocities = neighbors[:, 2:]
    f_sep = ((position - neighbor_positions) / np.linalg.norm(position - neighbor_positions, axis=1, keepdims=True)).sum(axis=0) / neighbors.shape[0]
    f_align = (neighbor_velocities - velocity).mean(axis=0)
    f_coh = neighbor_positions.mean(axis=0) - position
    return u_scale * (w_sep * f_sep + w_align * f_align + w_coh * f_coh)
