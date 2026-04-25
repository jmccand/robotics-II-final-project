from abc import ABC, abstractmethod

import matplotlib.patches
import numpy as np


class Shape(ABC):
    @abstractmethod
    def contains(self, point: np.ndarray) -> bool: ...

    @abstractmethod
    def distance(self, point: np.ndarray) -> float: ...

    @abstractmethod
    def closest_point(self, point: np.ndarray) -> np.ndarray: ...

    @abstractmethod
    def to_patch(self) -> matplotlib.patches.Patch: ...


class Circle(Shape):
    def __init__(self, center: np.ndarray, radius: float):
        self.center = np.asarray(center, dtype=float)
        self.radius = float(radius)

    def contains(self, point: np.ndarray) -> bool:
        return np.linalg.norm(point - self.center) <= self.radius

    def distance(self, point: np.ndarray) -> float:
        return max(0.0, np.linalg.norm(point - self.center) - self.radius)

    def closest_point(self, point: np.ndarray) -> np.ndarray:
        d = point - self.center
        n = np.linalg.norm(d)
        if n == 0:
            return self.center + np.array([self.radius, 0.0])
        return self.center + (d / n) * self.radius

    def to_patch(self) -> matplotlib.patches.Patch:
        return matplotlib.patches.Circle(self.center, self.radius, color="red", alpha=0.4)


