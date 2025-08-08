## 9‑State Drone EKF Simulation (MATLAB)

This project simulates a 6‑DOF multirotor with a 9‑state model and a sensor‑only Extended Kalman Filter (EKF). The EKF predicts using only IMU data and fuses GPS, barometer, and magnetometer measurements. A random‑walk velocity controller drives the "true" dynamics for a realistic trajectory.

### Contents
- `main_random.m`: Runs the end‑to‑end simulation, logging and playback.
- `parameters.m`: Central configuration (timing, vehicle, sensor noise, EKF tuning).
- `drone_dynamics.m`: Nonlinear 9‑state truth model with simplified cross‑coupling.
- `sensor_model.m`: Generates noisy IMU/GPS/Baro/Mag measurements from truth.
- `ekf_sensor_only.m`: EKF with IMU‑only prediction and sensor updates.
- `drone_dynamics_imu.m`: Strapdown mechanization used in EKF prediction.
- `calculate_F_sensor_only.m`: State Jacobian used in EKF covariance update.
- `rotation_matrix.m`: ZYX yaw‑pitch‑roll rotation helper.

### Quick start
1) Open MATLAB.
2) Set the working folder to the project root.
3) Open `parameters.m` and select a profile: set `profile = 'QAV250'` or `'S500'`.
4) Run `main_random.m`.

You should see console progress during the run, followed by an animation and optional analysis plots.

### Simulation overview
- State vector: `[x y z vx vy vz roll pitch yaw]` in NED, radians for angles.
- Truth integration: `drone_dynamics.m` uses thrust and torques computed from a smooth velocity command in `main_random.m`.
- Sensors: `sensor_model.m` outputs IMU (specific force, body rates), GPS (position), barometer (altitude = −z), magnetometer (yaw).
- EKF: `ekf_sensor_only.m` predicts via `drone_dynamics_imu.m` (IMU only) and updates per sensor at their own sample rates. The linearized transition `F` comes from `calculate_F_sensor_only.m`.

### Key tunables
- Timing: `params.Ts.physics`, `params.Ts.IMU`, `params.Ts.GPS`, `params.Ts.Baro`, `params.Ts.Mag`, `params.sim_duration`.
- Vehicle: `params.mass`, `params.I` (set via profile in `parameters.m`).
- EKF process noise: `params.Q` (diagonal 9×9, scaled by `dt` in code).
- Measurement noise: `params.R_gps`, `params.R_baro`, `params.R_mag`.

### How it fits together
1) `main_random.m` loads `parameters.m`, initializes state/covariance, and iterates over time.
2) It generates a smooth random‑walk target velocity (Ornstein–Uhlenbeck), converts to thrust/torques, and integrates the truth model.
3) `sensor_model.m` produces measurements with noise/bias.
4) `ekf_sensor_only.m` runs: predict at IMU rate using `drone_dynamics_imu.m`, and update with GPS/Baro/Mag at their rates.
5) Results are logged, animated, and optionally analyzed.

### Notes and conventions
- Frames: Body (b) and NED (n). `rotation_matrix.m` maps body→NED via ZYX.
- Barometer returns altitude, which is `−z` in NED convention.
- Multiple numerical safety checks (regularization, SVD conditioning, angle clamps) keep the simulation stable.

### Troubleshooting
- If the animation or analysis functions are missing in your MATLAB path, comment out those calls in `main_random.m` or add your own plotting. The core simulation and EKF will still run.
- If you observe NaN/Inf warnings, try reducing `params.sim_duration`, loosening `params.Q`, or lowering controller gains in `main_random.m`.

### License
No explicit license provided. Use responsibly within your organization or add a license file as needed.


