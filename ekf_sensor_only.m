function [x_est, P] = ekf_sensor_only(x_est, P, imu, z, params, dt, sensor_type)
%% PURPOSE
% Extended Kalman Filter that uses IMU-only prediction and selected sensor updates.
% This version does NOT use control inputs; suitable when actuators are unknown.
%
% INPUTS
% - x_est      : prior state estimate [pos(3); vel(3); att(3)]
% - P          : prior covariance (9x9)
% - imu        : [accel_meas(3); gyro_meas(3)] in body frame
% - z          : measurement vector/scalar (depends on sensor_type)
% - params     : struct with Q, R matrices and other config
% - dt         : time step (s)
% - sensor_type: 'IMU' | 'GPS' | 'Baro' | 'Mag'
%
% OUTPUTS
% - x_est, P   : posterior state and covariance after predict/update
%
% MAJOR STEPS
% 1) Validate and clamp inputs to safe ranges
% 2) Predict state using IMU mechanization; build Jacobian F and process noise Q
% 3) Predict covariance and condition it
% 4) Sensor-specific update step (optional for 'IMU')
% 5) Final validation, clamping, and conditioning of outputs

%% 1) Input validation and clamping of prior
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

%% 2) Nonlinear prediction using IMU mechanization only
try
    x_pred = x_est + drone_dynamics_imu(0, x_est, imu, params) * dt;
catch ME
    warning('EKF: Error in IMU mechanization. Using simple integration.');
    x_pred = x_est + [x_est(4:6); zeros(3,1); zeros(3,1)] * dt;
end

% Clamp predicted attitude to realistic range to avoid transient divergence
x_pred(7:9) = max(min(x_pred(7:9), deg2rad(60)), -deg2rad(60));

% Validate prediction
if any(~isfinite(x_pred))
    warning('EKF: Invalid prediction. Using previous state.');
    x_pred = x_est;
end

% Simple Jacobian for sensor-only EKF (no thrust/torque inputs)
F = calculate_F_sensor_only(x_est, imu, dt);

% Process noise (sensor-only, no control inputs)
Q = params.Q * dt;

% Ensure Q is positive definite via SVD
[U, S, V] = svd(Q);
S = max(S, 1e-12);
Q = U * S * V';

%% 3) Covariance prediction
P_pred = F*P*F' + Q;

% Condition P_pred
[U, S, V] = svd(P_pred);
S = max(S, 1e-12); % Prevent singular values from becoming too small
S = min(S, 1e6);   % Prevent singular values from becoming too large
P_pred = U * S * V';

switch sensor_type
    case 'IMU'
        %% 4) Sensor update: IMU-only -> skip update
        x_est = x_pred;
        P = P_pred;
    case 'GPS'
        % GPS measures position
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
        % Barometer measures altitude = -z (NED -> altitude)
        H = [0 0 -1 zeros(1,6)];
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
        % Magnetometer measures yaw angle psi
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

%% 5) Final validation, bounds, and conditioning
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