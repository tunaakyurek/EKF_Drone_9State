function [x_est, P] = ekf(x_est, P, imu, z, params, dt, sensor_type)
% ekf - Extended Kalman Filter for 9-state drone (pos, vel, att)
%   x_est: prior state estimate
%   P: prior covariance
%   imu: [accel_meas; gyro_meas] (body frame)
%   z: measurement
%   params: parameters struct
%   dt: timestep
%   sensor_type: 'IMU', 'GPS', 'Baro', 'Mag'

% Nonlinear prediction using IMU mechanization
x_pred = x_est + drone_dynamics_imu(0, x_est, imu, params) * dt;
F = calculate_F_jacobian(x_est, imu, dt);
Q = params.Q;
P_pred = F*P*F' + Q*dt;

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
        S = S + 1e-6 * eye(size(S));
        K = P_pred*H'/S;
        x_est = x_pred + K*y;
        P = (eye(9) - K*H)*P_pred;
    case 'Baro'
        H = [0 0 -1 zeros(1,6)]; % Barometer measures -z
        R = params.R_baro;
        y = z - H*x_pred;
        S = H*P_pred*H' + R;
        S = S + 1e-6 * eye(size(S));
        K = P_pred*H'/S;
        x_est = x_pred + K*y;
        P = (eye(9) - K*H)*P_pred;
    case 'Mag'
        H = [zeros(1,8), 1];
        R = params.R_mag;
        y = z - H*x_pred;
        S = H*P_pred*H' + R;
        S = S + 1e-6 * eye(size(S));
        K = P_pred*H'/S;
        x_est = x_pred + K*y;
        P = (eye(9) - K*H)*P_pred;
    otherwise
        error('Unknown sensor type');
end
end 