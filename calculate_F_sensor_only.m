function F = calculate_F_sensor_only(x, imu, dt)
% calculate_F_sensor_only - Jacobian for sensor-only EKF (no control inputs)
%   x: state vector [pos; vel; att]
%   imu: [accel_meas; gyro_meas] (body frame)
%   dt: timestep

% Extract state components
pos = x(1:3);
vel = x(4:6);
att = x(7:9); % [phi; theta; psi]

% Extract IMU measurements
if length(imu) >= 6
    accel_meas = imu(1:3);
    gyro_meas = imu(4:6);
else
    accel_meas = zeros(3,1);
    gyro_meas = zeros(3,1);
end

% Identity matrices
I3 = eye(3);
Z3 = zeros(3);

% Simple Jacobian for sensor-only EKF
% This models the effect of current state on the next state
% without requiring control inputs

% Position: x(k+1) = x(k) + v(k)*dt
% Velocity: v(k+1) = v(k) + a(k)*dt (where a comes from IMU)
% Attitude: att(k+1) = att(k) + omega(k)*dt (where omega comes from IMU)

% For position: d/dx = I, d/dv = dt*I, d/datt = 0
% For velocity: d/dx = 0, d/dv = I, d/datt = 0 (simplified)
% For attitude: d/dx = 0, d/dv = 0, d/datt = I (simplified)

F = [
    I3, dt*I3, Z3;    % Position row
    Z3, I3,    Z3;    % Velocity row (simplified)
    Z3, Z3,    I3     % Attitude row (simplified)
];

% Add small regularization to prevent singularity
F = F + 1e-12 * eye(9);

% Ensure numerical stability
[U, S, V] = svd(F);
S = max(S, 1e-12);
S = min(S, 1e6);
F = U * S * V';
end 