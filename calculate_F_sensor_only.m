function F = calculate_F_sensor_only(x, imu, dt)
%% PURPOSE
% Jacobian (state-transition linearization) for sensor-only EKF prediction.
% No control inputs are used; IMU drives the prediction model.
%
% INPUTS
% - x   : state vector [pos(3); vel(3); att(3)] with attitude [phi; theta; psi]
% - imu : [accel_meas(3); gyro_meas(3)] in body frame
% - dt  : timestep (s)
%
% OUTPUTS
% - F   : 9x9 discrete-time Jacobian used in covariance time-update
%
% MAJOR STEPS
% 1) Extract state and IMU components
% 2) Build a simple constant-velocity/constant-attitude-rate Jacobian
% 3) Add tiny regularization for numerical safety
% 4) Clamp singular values via SVD to keep F well-conditioned
%
% Coordinate frames: position/velocity are in NED; IMU is body frame.

% Original details retained below
% calculate_F_sensor_only - Jacobian for sensor-only EKF (no control inputs)
%   x: state vector [pos; vel; att]
%   imu: [accel_meas; gyro_meas] (body frame)
%   dt: timestep

%% 1) Extract state components
pos = x(1:3);
vel = x(4:6);
att = x(7:9); % [phi; theta; psi]

%% 2) Extract IMU measurements (fallback to zeros if not provided)
if length(imu) >= 6
    accel_meas = imu(1:3);
    gyro_meas = imu(4:6);
else
    accel_meas = zeros(3,1);
    gyro_meas = zeros(3,1);
end

%% Utilities
I3 = eye(3);
Z3 = zeros(3);

%% 3) Simple Jacobian for sensor-only EKF (no explicit inputs)
% Models how current state influences next state under IMU-driven motion

% Position: x(k+1) = x(k) + v(k)*dt
% Velocity: v(k+1) = v(k) + a(k)*dt (a from IMU)
% Attitude: att(k+1) = att(k) + omega(k)*dt (omega from IMU)

% For position: d/dx = I, d/dv = dt*I, d/datt = 0
% For velocity: d/dx = 0, d/dv = I, d/datt = 0 (simplified)
% For attitude: d/dx = 0, d/dv = 0, d/datt = I (simplified)

F = [
    I3, dt*I3, Z3;    % Position row
    Z3, I3,    Z3;    % Velocity row (simplified)
    Z3, Z3,    I3     % Attitude row (simplified)
];

%% 4) Regularization and conditioning safeguards
% Add small regularization to prevent singularity
F = F + 1e-12 * eye(9);

% Ensure numerical stability via singular-value clamping
[U, S, V] = svd(F);
S = max(S, 1e-12);
S = min(S, 1e6);
F = U * S * V';
end 