"""
Basic unit tests for kalman_filter.py

Run with:
  python3 -m unittest test_kalman_filter.py
"""

import unittest
from kalman_filter import KalmanFilter1D, generate_noisy_signal, rmse


class TestKalmanFilter1D(unittest.TestCase):
    def test_converges_toward_constant_measurement(self):
        kf = KalmanFilter1D(process_variance=0.01, sensor_variance=1.0, initial_estimate=0.0)
        estimate = None
        for _ in range(200):
            estimate = kf.update(5.0)
        self.assertAlmostEqual(estimate, 5.0, delta=0.1)

    def test_error_estimate_shrinks_over_time(self):
        kf = KalmanFilter1D(process_variance=0.01, sensor_variance=1.0, initial_error=10.0)
        first_error = kf.error_estimate
        for _ in range(20):
            kf.update(1.0)
        self.assertLess(kf.error_estimate, first_error)

    def test_filter_smooths_noisy_constant_signal(self):
        # A noisy but constant signal: the filtered output should have
        # lower variance than the raw measurements.
        kf = KalmanFilter1D(process_variance=0.001, sensor_variance=4.0, initial_estimate=0.0)
        measurements = [0.0, 3.0, -2.0, 2.5, -1.5, 1.0, -0.5, 2.0, -1.0, 0.5] * 5
        filtered = [kf.update(m) for m in measurements]

        def variance(values):
            mean = sum(values) / len(values)
            return sum((v - mean) ** 2 for v in values) / len(values)

        self.assertLess(variance(filtered), variance(measurements))


class TestSignalGeneration(unittest.TestCase):
    def test_generate_noisy_signal_shapes_match(self):
        true_values, measurements = generate_noisy_signal(50, sensor_noise_std=1.0, seed=1)
        self.assertEqual(len(true_values), 50)
        self.assertEqual(len(measurements), 50)

    def test_zero_noise_means_measurements_equal_truth(self):
        true_values, measurements = generate_noisy_signal(30, sensor_noise_std=0.0, seed=1)
        for t, m in zip(true_values, measurements):
            self.assertAlmostEqual(t, m)


class TestRMSE(unittest.TestCase):
    def test_rmse_zero_for_identical_series(self):
        series = [1.0, 2.0, 3.0, 4.0]
        self.assertEqual(rmse(series, series), 0.0)

    def test_rmse_positive_for_different_series(self):
        a = [1.0, 2.0, 3.0]
        b = [2.0, 2.0, 2.0]
        self.assertGreater(rmse(a, b), 0.0)


if __name__ == "__main__":
    unittest.main()
