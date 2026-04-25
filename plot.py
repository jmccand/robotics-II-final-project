import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from formation import Formation
from path import Path


# accumulates simulation frames then renders animation and per-robot deviation plots
class Plot:
    def __init__(
        self,
        path: Path,
        formation: Formation,
        obstacles: list | None = None,
        boundary: float = 5.0,
        dropout_robot: int | None = None,
        dropout_frame: int = 0,
    ):
        self.path = path
        self.formation = formation
        self.obstacles = obstacles or []
        self.boundary = boundary
        self.dropout_robot = dropout_robot
        self.dropout_frame = dropout_frame
        self._states_history: list[np.ndarray] = []
        self._t_history: list[float] = []
        self._desired_history: list[np.ndarray] = []
        self._ideal_history: list[np.ndarray] = []
        self._anim: FuncAnimation | None = None
        self._fig: plt.Figure | None = None

    def record(
        self,
        states: np.ndarray,
        t: float,
        desired_positions: np.ndarray,
        ideal_positions: np.ndarray | None = None,
    ) -> None:
        self._states_history.append(states.copy())
        self._t_history.append(t)
        self._desired_history.append(desired_positions.copy())
        if ideal_positions is not None:
            self._ideal_history.append(ideal_positions.copy())
        else:
            self._ideal_history.append(desired_positions.copy())

    def animate(self, interval: int = 10) -> FuncAnimation:
        path_pts = np.array([self.path.point(s) for s in np.linspace(0, 1, 300)])
        margin = 2.0

        fig, (ax_global, ax_zoom) = plt.subplots(1, 2, figsize=(14, 7))
        self._fig = fig

        def _draw_robots(
            ax: plt.Axes, states: np.ndarray, desired: np.ndarray, ideal: np.ndarray, frame_idx: int, size: int = 200
        ) -> None:
            dropped = (
                self.dropout_robot is not None and frame_idx >= self.dropout_frame
            )
            ax.scatter(ideal[:, 0], ideal[:, 1], marker="x", c="green", s=100, alpha=0.5, zorder=2, label="reference")
            ax.scatter(desired[:, 0], desired[:, 1], marker="x", c="gray", s=60, zorder=3, label="desired")
            for i in range(len(states)):
                err = desired[i] - states[i, :2]
                mag = np.linalg.norm(err)
                if mag > 0.01:
                    display = min(mag, 2.0)
                    u, v = err / mag * display
                    ax.annotate(
                        "", xy=(states[i, 0] + u, states[i, 1] + v),
                        xytext=(states[i, 0], states[i, 1]),
                        arrowprops=dict(
                            arrowstyle="->", color="gray",
                            lw=min(0.5 + mag * 0.4, 2.5),
                            alpha=min(0.2 + mag * 0.15, 0.9),
                        ),
                        zorder=2,
                    )
            active = [i for i in range(len(states)) if not (dropped and i == self.dropout_robot)]
            if active:
                ax.scatter(states[active, 0], states[active, 1], c="royalblue", s=size, zorder=4)
            if dropped:
                d = self.dropout_robot
                ax.scatter([states[d, 0]], [states[d, 1]], c="lightgray", s=size, zorder=4,
                           edgecolors="gray", linewidths=1)
            for i, s in enumerate(states):
                ax.annotate(str(i), (s[0], s[1]), fontsize=7, fontweight="bold",
                            ha="center", va="center", color="white", zorder=5)

        def draw_frame(frame_idx: int) -> None:
            ax_global.clear()
            ax_zoom.clear()

            states = self._states_history[frame_idx]
            desired = self._desired_history[frame_idx]
            ideal = self._ideal_history[frame_idx]
            t_label = f"{frame_idx / max(len(self._states_history) - 1, 1):.1%} complete"

            # global view
            ax_global.set_xlim(-self.boundary, self.boundary)
            ax_global.set_ylim(-self.boundary, self.boundary)
            ax_global.set_aspect("equal")
            ax_global.set_title(t_label)
            for obs in self.obstacles:
                ax_global.add_patch(obs.to_patch())
            ax_global.plot(path_pts[:, 0], path_pts[:, 1], "r--", lw=1, alpha=0.5, label="path")
            _draw_robots(ax_global, states, desired, ideal, frame_idx, size=100)
            ax_global.legend(loc="upper left", fontsize=7)

            # zoomed view: fit to current robot positions
            xs, ys = states[:, 0], states[:, 1]
            cx, cy = xs.mean(), ys.mean()
            half = max((xs.max() - xs.min()) / 2, (ys.max() - ys.min()) / 2) + margin
            ax_zoom.set_xlim(cx - half, cx + half)
            ax_zoom.set_ylim(cy - half, cy + half)
            ax_zoom.set_aspect("equal")
            ax_zoom.set_title(f"zoom  {t_label}")
            for obs in self.obstacles:
                ax_zoom.add_patch(obs.to_patch())
            ax_zoom.plot(path_pts[:, 0], path_pts[:, 1], "r--", lw=1, alpha=0.5, label="path")
            _draw_robots(ax_zoom, states, desired, ideal, frame_idx, size=200)
            ax_zoom.legend(loc="upper left", fontsize=7)

        self._anim = FuncAnimation(
            fig, draw_frame, frames=len(self._states_history), interval=interval
        )
        return self._anim

    def save(self, filename: str) -> None:
        if self._anim is None:
            self.animate()
        if filename.endswith(".gif"):
            self._anim.save(filename, writer="pillow")
        else:
            self._anim.save(filename, writer="ffmpeg")
        print(f"Saved animation to {filename}")

    def plot_deviations(self) -> plt.Figure:
        n_robots = self._states_history[0].shape[0]
        n_frames = len(self._states_history)

        deviations = np.zeros((n_frames, n_robots))
        for f, (states, ideal) in enumerate(
            zip(self._states_history, self._ideal_history)
        ):
            for i in range(n_robots):
                deviations[f, i] = np.linalg.norm(states[i, :2] - ideal[i])

        fig, ax = plt.subplots(figsize=(10, 4))
        for i in range(n_robots):
            ax.plot(deviations[:, i], label=f"Robot {i}")
        ax.set_xlabel("Frame")
        ax.set_ylabel("Deviation from slot (units)")
        ax.legend()
        ax.grid(True, alpha=0.3)
        return fig
