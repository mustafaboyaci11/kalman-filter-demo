#!/usr/bin/env python3
"""
kalman-filter-demo

A minimal 1D Kalman filter, demonstrated on a noisy sensor signal.
This is the kind of filter used constantly in control and embedded
systems to combine a noisy sensor reading with a prediction of
where the system should be, producing a smoother, more trustworthy
estimate than either one alone.

No external dependencies -- pure Python standard library, so it
runs fine next to embedded toolchains.

Usage:
  python3 kalman_filter.py
  python3 kalman_filter.py --process-noise 0.01 --sensor-noise 4.0
  python3 kalman_filter.py --samples 200 --seed 7
"""

import argparse
import math
import random


class KalmanFilter1D:
    """A simple scalar Kalman filter for tracking a slowly-varying
    true value from noisy measurements of it."""

    def __init__(self, process_variance, sensor_variance, initial_estimate=0.0, initial_error=1.0):
        self.process_variance = process_variance
        self.sensor_variance = sensor_variance
        self.estimate = initial_estimate
        self.error_estimate = initial_error

    def update(self, measurement):
        # Prediction step: assume the value doesn't change on its own,
        # but our uncertainty about it grows a little each step.
        predicted_estimate = self.estimate
        predicted_error = self.error_estimate + self.process_variance

        # Update step: blend the prediction with the new measurement,
        # weighted by how much we trust each one.
        kalman_gain = predicted_error / (predicted_error + self.sensor_variance)
        self.estimate = predicted_estimate + kalman_gain * (measurement - predicted_estimate)
        self.error_estimate = (1 - kalman_gain) * predicted_error

        return self.estimate


def generate_noisy_signal(n, sensor_noise_std, seed=None):
    """Simulate a true signal (a slow sine wave) observed through
    noisy sensor readings, the way a real analog sensor might behave."""
    rng = random.Random(seed)
    true_values = []
    measurements = []
    for i in range(n):
        true_value = 10 + 3 * math.sin(i / 15.0)
        noisy_measurement = true_value + rng.gauss(0, sensor_noise_std)
        true_values.append(true_value)
        measurements.append(noisy_measurement)
    return true_values, measurements


def rmse(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)) / len(a))


def ascii_plot(series_list, labels, width=70, height=15):
    all_values = [v for series in series_list for v in series]
    y_min, y_max = min(all_values), max(all_values)
    if y_max == y_min:
        y_max += 1.0

    n = len(series_list[0])
    sample_indices = [int(i * (n - 1) / (width - 1)) for i in range(width)]
    chars = ["#", "o", "."]

    grid = [[" " for _ in range(width)] for _ in range(height + 1)]
    for series_idx, series in enumerate(series_list):
        char = chars[series_idx % len(chars)]
        for col, idx in enumerate(sample_indices):
            value = series[idx]
            row = height - int((value - y_min) / (y_max - y_min) * height)
            row = max(0, min(height, row))
            grid[row][col] = char

    print(f"y-axis: {y_min:.2f} (bottom) to {y_max:.2f} (top)")
    for row in grid:
        print("|" + "".join(row))
    print("+" + "-" * width)
    legend = "  ".join(f"{chars[i]} = {label}" for i, label in enumerate(labels))
    print(legend)


def main():
    parser = argparse.ArgumentParser(description="Demonstrate a 1D Kalman filter on a noisy sensor signal.")
    parser.add_argument("--samples", type=int, default=150, help="Number of samples to simulate (default: 150)")
    parser.add_argument("--process-noise", type=float, default=0.02, help="Process variance Q (default: 0.02)")
    parser.add_argument("--sensor-noise", type=float, default=2.0, help="Sensor noise std deviation (default: 2.0)")
    parser.add_argument("--seed", type=int, default=None, help="Random seed for reproducible noise")
    args = parser.parse_args()

    sensor_variance = args.sensor_noise ** 2
    true_values, measurements = generate_noisy_signal(args.samples, args.sensor_noise, seed=args.seed)

    kf = KalmanFilter1D(args.process_noise, sensor_variance, initial_estimate=measurements[0])
    filtered = [kf.update(m) for m in measurements]

    print(f"Simulated {args.samples} samples, process_variance={args.process_noise}, sensor_noise_std={args.sensor_noise}")
    print()
    ascii_plot([measurements, filtered, true_values], ["raw", "filtered", "true"])
    print()
    print(f"RMSE raw vs true:      {rmse(measurements, true_values):.3f}")
    print(f"RMSE filtered vs true: {rmse(filtered, true_values):.3f}")


if __name__ == "__main__":
    main()
