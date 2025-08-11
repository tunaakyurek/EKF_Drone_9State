function [x_out, P_out, F_out, x_pred, P_pred] = ekf9_sensor_only_step(x_in, P_in, imu, params, dt)
% ekf9_sensor_only_step - IMU-driven predict for 9-state EKF with Jacobian output
% State: x = [pos(3); vel(3); att(3)]

% Predict state using existing mechanization
x_pred = x_in + drone_dynamics_imu(0, x_in, imu, params) * dt;
x_pred(7:9) = wrapToPi(x_pred(7:9));

% Linearize via existing helper
F_out = calculate_F_sensor_only(x_in, imu, dt);

% Process noise
Q = params.Q * dt;

% Covariance predict
P_pred = F_out * P_in * F_out' + Q;

% Condition P
[U, S, V] = svd(P_pred);
S = max(S, 1e-12);
S = min(S, 1e6);
P_pred = U * S * V';

x_out = x_pred;
P_out = P_pred;
end


