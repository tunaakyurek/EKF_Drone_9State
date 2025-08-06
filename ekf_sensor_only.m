function [x_est, P] = ekf_sensor_only(x_est, P, imu, z, params, dt, sensor_type)
% ekf_sensor_only - Sensor-only Extended Kalman Filter for 9-state drone
%   x_est: prior state estimate [pos; vel; att]
%   P: prior covariance
%   imu: [accel_meas; gyro_meas] (body frame)
%   z: measurement
%   params: parameters struct
%   dt: timestep
%   sensor_type: 'IMU', 'GPS', 'Baro', 'Mag'

% Input validation
if any(~isfinite(x_est)) || any(~isfinite(P(:)))
    warning('EKF: Invalid input state or covariance. Using safe defaults.');
    x_est = zeros(9,1);
    P = 0.1*eye(9);
end

% Bounds for state variables
pos_lim = 1000;    % Position limit (m)
vel_lim = 50;      % Velocity limit (m/s)
att_lim = pi/2;    % Attitude limit (rad)

% Clamp state to reasonable bounds
x_est(1:3) = max(min(x_est(1:3), pos_lim), -pos_lim);  % Position
x_est(4:6) = max(min(x_est(4:6), vel_lim), -vel_lim);  % Velocity
x_est(7:9) = max(min(x_est(7:9), att_lim), -att_lim);  % Attitude

% Nonlinear prediction using IMU mechanization only
try
    x_pred = x_est + drone_dynamics_imu(0, x_est, imu, params) * dt;
catch ME
    warning('EKF: Error in IMU mechanization. Using simple integration.');
    x_pred = x_est + [x_est(4:6); zeros(3,1); zeros(3,1)] * dt;
end

% Validate prediction
if any(~isfinite(x_pred))
    warning('EKF: Invalid prediction. Using previous state.');
    x_pred = x_est;
end

% Simple Jacobian for sensor-only EKF (no thrust/torque inputs)
F = calculate_F_sensor_only(x_est, imu, dt);

% Process noise (sensor-only, no control inputs)
Q = params.Q * dt;

% Ensure Q is positive definite
[U, S, V] = svd(Q);
S = max(S, 1e-12);
Q = U * S * V';

% Covariance prediction
P_pred = F*P*F' + Q;

% Ensure P_pred is positive definite and well-conditioned
[U, S, V] = svd(P_pred);
S = max(S, 1e-12); % Prevent singular values from becoming too small
S = min(S, 1e6);   % Prevent singular values from becoming too large
P_pred = U * S * V';

switch sensor_type
    case 'IMU'
        % No update, only prediction
        x_est = x_pred;
        P = P_pred;
    case 'GPS'
        H = [eye(3), zeros(3,6)];
        R = params.R_gps;
        y = z - H*x_pred;
        S = H*P_pred*H' + R;
        
        % Add regularization to prevent singularity
        S = S + 1e-6 * eye(size(S));
        
        % Check condition number
        if cond(S) > 1e12
            warning('EKF: GPS innovation covariance is ill-conditioned. Skipping update.');
            x_est = x_pred;
            P = P_pred;
        else
            K = P_pred*H'/S;
            x_est = x_pred + K*y;
            P = (eye(9) - K*H)*P_pred;
        end
    case 'Baro'
        H = [0 0 -1 zeros(1,6)]; % Barometer measures -z
        R = params.R_baro;
        y = z - H*x_pred;
        S = H*P_pred*H' + R;
        
        % Add regularization
        S = S + 1e-6;
        
        if cond(S) > 1e12
            warning('EKF: Baro innovation covariance is ill-conditioned. Skipping update.');
            x_est = x_pred;
            P = P_pred;
        else
            K = P_pred*H'/S;
            x_est = x_pred + K*y;
            P = (eye(9) - K*H)*P_pred;
        end
    case 'Mag'
        H = [zeros(1,8), 1];
        R = params.R_mag;
        y = z - H*x_pred;
        S = H*P_pred*H' + R;
        
        % Add regularization
        S = S + 1e-6;
        
        if cond(S) > 1e12
            warning('EKF: Mag innovation covariance is ill-conditioned. Skipping update.');
            x_est = x_pred;
            P = P_pred;
        else
            K = P_pred*H'/S;
            x_est = x_pred + K*y;
            P = (eye(9) - K*H)*P_pred;
        end
    otherwise
        error('Unknown sensor type');
end

% Final validation and bounds enforcement
if any(~isfinite(x_est)) || any(~isfinite(P(:)))
    warning('EKF: Final state or covariance contains NaN/Inf. Resetting to prediction.');
    x_est = x_pred;
    P = P_pred;
end

% Clamp final state to reasonable bounds
x_est(1:3) = max(min(x_est(1:3), pos_lim), -pos_lim);  % Position
x_est(4:6) = max(min(x_est(4:6), vel_lim), -vel_lim);  % Velocity
x_est(7:9) = max(min(x_est(7:9), att_lim), -att_lim);  % Attitude

% Ensure final covariance is positive definite
[U, S, V] = svd(P);
S = max(S, 1e-12);
S = min(S, 1e6);
P = U * S * V';
end 