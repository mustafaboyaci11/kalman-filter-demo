# kalman-filter-demo

A minimal 1D Kalman filter, demonstrated on a noisy simulated sensor signal, with an ASCII plot and RMSE comparison so you can see the filter actually working from the terminal.

#### Why
A Kalman filter is one of the most-used tools in control and embedded systems: it blends a noisy sensor reading with a prediction of where the system should be, producing an estimate that's smoother and more trustworthy than the raw sensor alone. This is a from-scratch, dependency-free implementation meant for learning and quick experiments, not a production filtering library.

#### Requirements
Python 3.7+, no external dependencies — pure standard library.

#### Usage
Run with default settings:
```
python3 kalman_filter.py
```

Adjust the filter tuning or the simulated noise:
```
python3 kalman_filter.py --process-noise 0.01 --sensor-noise 4.0
```

Use a fixed seed for reproducible output:
```
python3 kalman_filter.py --samples 200 --seed 7
```

Run the test suite:
```
python3 -m unittest test_kalman_filter.py
```

#### Example output
```
Simulated 150 samples, process_variance=0.02, sensor_noise_std=2.0

y-axis: 4.13 (bottom) to 17.13 (top)
|                                                     #
|        #                                        #
|      #  .....   ##      #                        #  ..... #
|     #...# ooo....o #                            .... ##o ...oooo
...
+----------------------------------------------------------------------
# = raw  o = filtered  . = true

RMSE raw vs true:      1.868
RMSE filtered vs true: 1.316
```

The filtered line tracks the true signal noticeably more closely than the raw measurements, and the RMSE numbers confirm it.

#### How it works
The filter models the tracked value as roughly constant from one step to the next (a common, simple assumption for slowly-changing physical quantities like temperature or position). Each update:

1. **Predicts** — assumes the value hasn't changed, but grows the uncertainty slightly (`process_variance`) to account for the fact that it might have.
2. **Updates** — blends the prediction with the new noisy measurement, weighted by the Kalman gain: trust the measurement more when it's precise, trust the prediction more when the sensor is noisy.

The simulated signal itself is a slow sine wave with independent Gaussian noise added per sample, standing in for something like a slightly-drifting temperature or position reading.

#### Limitations
This is a constant-velocity-style 1D filter, not a full state-space implementation with matrices, control inputs, or multiple correlated states. It's meant to make the core predict/update idea intuitive, not to replace a proper library like `filterpy` for real multi-dimensional estimation problems.

#### Roadmap
- Extend to a 2D constant-velocity model (position + velocity) with vector/matrix math
- Add a mode that reads real sensor data from a CSV file instead of the simulator
- Add a live-updating terminal view for streaming data

#### Contributing
Issues and pull requests are welcome, especially a 2D/vector extension or real-world example datasets.
