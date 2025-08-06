function [x_est, P] = ekf_adaptive(x_est, P, imu, z, params, dt, sensor_type)
% ekf_adaptive - Adaptive Extended Kalman Filter for 9-state drone
%   x_est: prior state estimate
%   P: prior covariance
%   imu: [accel_meas; gyro_meas] (body frame)
%   z: measurement
%   params: parameters struct
%   dt: timestep
%   sensor_type: 'IMU', 'GPS', 'Baro', 'Mag'

persistent prev_angular_vel turn_detected noise_scale

if isempty(prev_angular_vel)
    prev_angular_vel = zeros(3,1);
    turn_detected = false;
    noise_scale = params.noise_scale_normal;
end

% Input validation and bounds checking
if any(~isfinite(x_est)) || any(~isfinite(P(:)))
    warning('Adaptive EKF: Invalid input state or covariance detected. Using safe defaults.');
    x_est = zeros(9,1);
    P = 0.1*eye(9);
end

% Bounds for state variables (reasonable physical limits)
pos_lim = 1000;    % Position limit (m)
vel_lim = 50;      % Velocity limit (m/s)
att_lim = pi/2;    % Attitude limit (rad)

% Clamp state to reasonable bounds
x_est(1:3) = max(min(x_est(1:3), pos_lim), -pos_lim);  % Position
x_est(4:6) = max(min(x_est(4:6), vel_lim), -vel_lim);  % Velocity
x_est(7:9) = max(min(x_est(7:9), att_lim), -att_lim);  % Attitude

% Nonlinear prediction using IMU mechanization
try
    x_pred = x_est + drone_dynamics_imu(0, x_est, imu, params) * dt;
    F = calculate_F_jacobian(x_est, imu, dt);
catch ME
    warning('Adaptive EKF: Error in prediction step. Using simple integration.');
    x_pred = x_est + [x_est(4:6); zeros(3,1); zeros(3,1)] * dt;
    F = eye(9);
end

% Validate prediction
if any(~isfinite(x_pred))
    warning('Adaptive EKF: Invalid prediction. Using previous state.');
    x_pred = x_est;
    F = eye(9);
end

% Adaptive noise scaling based on turn detection
if params.adaptive_noise
    % Extract angular velocity from IMU
    angular_vel = imu(4:6);
    if length(angular_vel) >= 3
        angular_accel = (angular_vel - prev_angular_vel) / dt;
        
        % Detect turns based on angular velocity magnitude
        if norm(angular_vel) > params.turn_threshold || norm(angular_accel) > params.turn_threshold
            turn_detected = true;
            noise_scale = params.noise_scale_turn;
        else
            turn_detected = false;
            noise_scale = params.noise_scale_normal;
        end
        
        prev_angular_vel = angular_vel;
    else
        noise_scale = params.noise_scale_normal;
    end
    
    % Scale Q matrix during turns
    Q_scaled = params.Q * noise_scale;
else
    Q_scaled = params.Q;
end

% Ensure Q is positive definite
[U, S, V] = svd(Q_scaled);
S = max(S, 1e-12);
Q_scaled = U * S * V';

% Covariance prediction with regularization
P_pred = F*P*F' + Q_scaled*dt;

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
        
        % Check for numerical issues in innovation
        if any(~isfinite(y)) || any(~isfinite(S(:)))
            warning('Adaptive EKF: Invalid GPS innovation. Skipping update.');
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
        
        if any(~isfinite(y)) || any(~isfinite(S(:)))
            warning('Adaptive EKF: Invalid baro innovation. Skipping update.');
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
        
        if any(~isfinite(y)) || any(~isfinite(S(:)))
            warning('Adaptive EKF: Invalid mag innovation. Skipping update.');
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
    warning('Adaptive EKF: Final state or covariance contains NaN/Inf. Resetting to prediction.');
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