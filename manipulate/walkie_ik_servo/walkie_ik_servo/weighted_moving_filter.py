"""Weighted moving average filter for smoothing robot joint trajectories."""

import numpy as np
from typing import List, Optional


class WeightedMovingFilter:
    """Weighted moving average filter for smoothing control signals.

    Applies a weighted convolution over a sliding window of joint position
    samples to produce smooth motion commands.

    Args:
        weights: Weight array for the moving average. First element corresponds
                 to the most recent sample. Must sum to 1.0.
        data_size: Number of channels (joints) to filter.
    """

    def __init__(self, weights: List[float], data_size: int = 7):
        self.weights = np.array(weights, dtype=np.float64)
        self.window_size = len(weights)
        self.data_size = data_size
        self.queue: List[np.ndarray] = []
        self.filtered_data = np.zeros(data_size, dtype=np.float64)

    def _apply_filter(self) -> np.ndarray:
        """Apply weighted moving average across the sample window."""
        if len(self.queue) < self.window_size:
            # Not enough samples yet, return latest
            return self.queue[-1].copy()

        result = np.zeros(self.data_size, dtype=np.float64)
        data_matrix = np.array(self.queue[-self.window_size :])  # (window, channels)

        for ch in range(self.data_size):
            result[ch] = np.convolve(data_matrix[:, ch], self.weights, mode="valid")[0]

        return result

    def add_data(self, new_data: np.ndarray) -> None:
        """Add a new sample and update the filtered output.

        Skips duplicate consecutive samples.

        Args:
            new_data: Joint position array of shape (data_size,)
        """
        new_data = np.asarray(new_data, dtype=np.float64)

        # Skip if identical to last sample
        if len(self.queue) > 0 and np.array_equal(new_data, self.queue[-1]):
            return

        self.queue.append(new_data.copy())

        # Keep only the window we need
        if len(self.queue) > self.window_size * 2:
            self.queue = self.queue[-self.window_size :]

        self.filtered_data = self._apply_filter()

    def reset(self) -> None:
        """Clear all samples and reset filtered output to zeros."""
        self.queue.clear()
        self.filtered_data = np.zeros(self.data_size, dtype=np.float64)

    @property
    def is_ready(self) -> bool:
        """Whether the filter window is fully populated."""
        return len(self.queue) >= self.window_size
