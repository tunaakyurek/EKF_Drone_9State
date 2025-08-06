function [x_est, P, sensor_updates] = enhanced_sensor_fusion(x_est, P, sensors, config, params, dt, k)
% enhanced_sensor_fusion - Realistic sensor-only EKF fusion system
% Based on MATLAB Simulink Autonomous Flight Design Tutorial methodology
%
% STRICT POLICY: Uses ONLY realistic sensor measurements, NO true values
%
% Implements enhanced EKF with:
% - Multi-rate sensor processing (IMU: 200Hz, GPS: 20Hz, Baro: 20Hz, Mag: 20Hz)
% - Adaptive noise scaling during turns
% - Robust numerical handling
%
% Inputs:
%   x_est - Current state estimate [pos(3); vel(3); att(3)]
%   P - Current covariance matrix
%   sensors - Sensor measurements struct (REALISTIC DATA ONLY)
%   config - Sensor fusion configuration
%   params - System parameters
%   dt - Timestep
%   k - Current simulation step
%
% Outputs:
%   x_est - Updated state estimate
%   P - Updated covariance matrix
%   sensor_updates - Structure indicating which sensors were processed

% Initialize sensor update flags
sensor_updates = struct();
sensor_updates.imu_updated = false;
sensor_updates.gps_updated = false;
sensor_updates.baro_updated = false;
sensor_updates.mag_updated = false;

% Input validation and safety bounds
x_est = enforce_state_bounds(x_est);
P = ensure_positive_definite(P);

% Main sensor fusion processing - ENHANCED EKF ONLY
[x_est, P, sensor_updates] = enhanced_ekf_fusion(x_est, P, sensors, params, dt, k, config);

% Final state validation and bounds enforcement
x_est = enforce_state_bounds(x_est);
P = ensure_positive_definite(P);

end

%% ENHANCED EKF FUSION (Primary Method)
function [x_est, P, sensor_updates] = enhanced_ekf_fusion(x_est, P, sensors, params, dt, k, config)
% Enhanced EKF implementation following tutorial methodology
% USES ONLY REALISTIC SENSOR DATA - NO TRUE VALUES

sensor_updates = struct('imu_updated', false, 'gps_updated', false, ...
                       'baro_updated', false, 'mag_updated', false);

try
    %% IMU PREDICTION STEP (High Frequency - 200 Hz)
    if mod(k-1, round(params.Ts.IMU/dt)) == 0
        % Enhanced IMU mechanization with improved integration
        imu_meas = [sensors.accel; sensors.gyro];
        
        % Nonlinear prediction using enhanced IMU dynamics
        x_pred = x_est + enhanced_imu_mechanization(x_est, imu_meas, params) * dt;
        
        % Enhanced Jacobian calculation for sensor-only EKF
        F = calculate_enhanced_jacobian(x_est, imu_meas, dt);
        
        % Adaptive process noise based on motion dynamics
        Q = calculate_adaptive_process_noise(x_est, imu_meas, params, dt);
        
        % Covariance prediction with enhanced numerical stability
        P = F*P*F' + Q;
        P = ensure_positive_definite(P);
        
        x_est = x_pred;
        sensor_updates.imu_updated = true;
    end
    
    %% GPS CORRECTION (20 Hz)
    if mod(k-1, round(params.Ts.GPS/dt)) == 0 && isfield(sensors, 'gps')
        [x_est, P] = gps_measurement_update(x_est, P, sensors.gps, params);
        sensor_updates.gps_updated = true;
    end
    
    %% BAROMETER CORRECTION (20 Hz)
    if mod(k-1, round(params.Ts.Baro/dt)) == 0 && isfield(sensors, 'baro')
        [x_est, P] = baro_measurement_update(x_est, P, sensors.baro, params);
        sensor_updates.baro_updated = true;
    end
    
    %% MAGNETOMETER CORRECTION (20 Hz)
    if mod(k-1, round(params.Ts.Mag/dt)) == 0 && isfield(sensors, 'mag')
        [x_est, P] = mag_measurement_update(x_est, P, sensors.mag, params);
        sensor_updates.mag_updated = true;
    end
    
    %% ZERO VELOCITY UPDATE (ZUPT) for enhanced robustness during GPS outages
    if mod(k-1, round(0.1/dt)) == 0 % Check every 100ms
        [x_est, P, zupt_applied] = zero_velocity_update(x_est, P, sensors, params);
        if zupt_applied
            sensor_updates.zupt_updated = true;
        end
    end
    
catch ME
    warning('Enhanced EKF: Error in sensor fusion. Using previous estimates.');
    warning('Error: %s', ME.message);
end

end

%% COMPLEMENTARY FILTER FUSION (Alternative Method)
function [x_est, P, sensor_updates] = complementary_filter_fusion(x_est, P, sensors, params, dt, k)
% Complementary filter implementation following tutorial methodology

sensor_updates = struct('imu_updated', false, 'gps_updated', false, ...
                       'baro_updated', false, 'mag_updated', false);

% Complementary filter gains (tunable)
alpha = 0.95; % High-pass gain for gyro (short-term accuracy)
beta = 0.05;  % Low-pass gain for accel/mag (long-term stability)

try
    %% IMU PROCESSING (High frequency)
    if mod(k-1, round(params.Ts.IMU/dt)) == 0
        % Gyroscope integration (high-pass filtered)
        gyro_integration = x_est(7:9) + sensors.gyro * dt;
        
        % Accelerometer-based attitude estimation (low-pass filtered)
        accel_attitude = estimate_attitude_from_accel(sensors.accel);
        
        % Complementary fusion for attitude
        x_est(7:8) = alpha * gyro_integration(1:2) + beta * accel_attitude(1:2);
        
        % Position and velocity updates with simple integration
        x_est(4:6) = x_est(4:6) + sensors.accel * dt; % Velocity
        x_est(1:3) = x_est(1:3) + x_est(4:6) * dt;    % Position
        
        sensor_updates.imu_updated = true;
    end
    
    %% GPS CORRECTION (Lower frequency)
    if mod(k-1, round(params.Ts.GPS/dt)) == 0 && isfield(sensors, 'gps')
        % Simple GPS correction for position
        gps_gain = 0.3; % Tunable gain
        x_est(1:3) = (1 - gps_gain) * x_est(1:3) + gps_gain * sensors.gps;
        sensor_updates.gps_updated = true;
    end
    
    %% MAGNETOMETER CORRECTION
    if mod(k-1, round(params.Ts.Mag/dt)) == 0 && isfield(sensors, 'mag')
        % Magnetometer correction for yaw
        mag_gain = 0.1;
        x_est(9) = (1 - mag_gain) * x_est(9) + mag_gain * sensors.mag;
        sensor_updates.mag_updated = true;
    end
    
catch ME
    warning('Complementary Filter: Error in processing. Using previous estimates.');
    warning('Error: %s', ME.message);
end

end

%% ENHANCED IMU MECHANIZATION
function x_dot = enhanced_imu_mechanization(x, imu_meas, params)
% Enhanced IMU mechanization following tutorial methodology

accel = imu_meas(1:3);
gyro = imu_meas(4:6);

% State extraction
pos = x(1:3);
vel = x(4:6);
att = x(7:9);

% Rotation matrix from body to NED
R = rotation_matrix(att(1), att(2), att(3));

% Acceleration in NED frame with gravity compensation
accel_ned = R * accel + params.g;

% Angular rates with enhanced stability
omega = enhanced_attitude_kinematics(att, gyro);

% State derivatives
x_dot = [vel; accel_ned; omega];

end

%% ENHANCED JACOBIAN CALCULATION
function F = calculate_enhanced_jacobian(x, imu_meas, dt)
% Enhanced Jacobian calculation for improved linearization

F = eye(9);

% Enhanced linearization around current state
vel = x(4:6);
att = x(7:9);
accel = imu_meas(1:3);
gyro = imu_meas(4:6);

% Position derivatives
F(1:3, 4:6) = eye(3) * dt; % dp/dv

% Velocity derivatives (accounting for attitude changes)
R = rotation_matrix(att(1), att(2), att(3));
[dR_dphi, dR_dtheta, dR_dpsi] = rotation_matrix_derivatives(att(1), att(2), att(3));

F(4, 7) = dt * (dR_dphi * accel)' * [1; 0; 0];  % dv_x/dphi
F(4, 8) = dt * (dR_dtheta * accel)' * [1; 0; 0]; % dv_x/dtheta
F(4, 9) = dt * (dR_dpsi * accel)' * [1; 0; 0];   % dv_x/dpsi

F(5, 7) = dt * (dR_dphi * accel)' * [0; 1; 0];   % dv_y/dphi
F(5, 8) = dt * (dR_dtheta * accel)' * [0; 1; 0]; % dv_y/dtheta
F(5, 9) = dt * (dR_dpsi * accel)' * [0; 1; 0];   % dv_y/dpsi

F(6, 7) = dt * (dR_dphi * accel)' * [0; 0; 1];   % dv_z/dphi
F(6, 8) = dt * (dR_dtheta * accel)' * [0; 0; 1]; % dv_z/dtheta
F(6, 9) = dt * (dR_dpsi * accel)' * [0; 0; 1];   % dv_z/dpsi

% Attitude dynamics linearization
E = euler_rate_matrix(att(1), att(2));
F(7:9, 7:9) = F(7:9, 7:9) + E * dt;

end

%% ADAPTIVE PROCESS NOISE
function Q = calculate_adaptive_process_noise(x, imu_meas, params, dt)
% IMPROVED: Calculate adaptive process noise with GPS outage handling

persistent last_gps_time gps_outage_duration;

% Initialize persistent variables
if isempty(last_gps_time)
    last_gps_time = 0;
    gps_outage_duration = 0;
end

% Base process noise
Q_base = params.Q * dt;

% Detect high dynamics scenarios
accel_magnitude = norm(imu_meas(1:3));
gyro_magnitude = norm(imu_meas(4:6));

% Scaling factors based on motion intensity
accel_scale = 1 + 0.5 * (accel_magnitude / 9.81); % Scale with acceleration
gyro_scale = 1 + 2.0 * gyro_magnitude;             % Scale with angular rates

% IMPROVED: GPS outage compensation
current_time = params.current_time; % Assume this is passed in params
if isfield(params, 'gps_available') && params.gps_available
    % GPS is available - reset outage tracking
    last_gps_time = current_time;
    gps_outage_duration = 0;
else
    % GPS is not available - track outage duration
    gps_outage_duration = current_time - last_gps_time;
end

% Inflate process noise during GPS outages
gps_outage_scale = 1.0;
if gps_outage_duration > 0
    % Exponentially increase uncertainty during GPS outage
    % After 10 seconds without GPS, 5x more position uncertainty
    gps_outage_scale = 1 + 4 * (1 - exp(-gps_outage_duration / 10));
end

% Apply adaptive scaling
Q = Q_base;
Q(1:3, 1:3) = Q(1:3, 1:3) * gps_outage_scale; % Position noise (GPS outage)
Q(4:6, 4:6) = Q(4:6, 4:6) * accel_scale * gps_outage_scale; % Velocity noise
Q(7:9, 7:9) = Q(7:9, 7:9) * gyro_scale;  % Attitude noise

% Ensure positive definiteness
Q = ensure_positive_definite(Q);

end

%% MEASUREMENT UPDATE FUNCTIONS
function [x_est, P] = gps_measurement_update(x_est, P, gps_meas, params)
% GPS measurement update with enhanced robustness

% Check for valid GPS data first
if any(isnan(gps_meas))
    % GPS not available - skip update
    return;
end

H_gps = [eye(3), zeros(3,6)]; % GPS measures position
R_gps = params.R_gps;

% Innovation
y = gps_meas - H_gps * x_est;

% Innovation covariance with regularization
S = H_gps * P * H_gps' + R_gps;
S = S + 1e-6 * eye(size(S)); % Regularization

% Check for reasonable innovation
if norm(y) < params.innovation_gate_gps % OPTIMIZED: Use tuned GPS innovation threshold
    % Kalman gain
    K = P * H_gps' / S;
    
    % State and covariance update
    x_est = x_est + K * y;
    P = (eye(9) - K * H_gps) * P;
    
    % Joseph form for numerical stability
    P = (eye(9) - K * H_gps) * P * (eye(9) - K * H_gps)' + K * R_gps * K';
else
    warning('GPS measurement rejected due to large innovation: %.1f m', norm(y));
end

end

function [x_est, P] = baro_measurement_update(x_est, P, baro_meas, params)
% Barometer measurement update

% Check for valid barometer data first
if isnan(baro_meas)
    % Barometer not available - skip update
    return;
end

H_baro = [0 0 -1 zeros(1,6)]; % Baro measures negative altitude
R_baro = params.R_baro;

% Innovation
y = baro_meas - H_baro * x_est;

% Innovation covariance
S = H_baro * P * H_baro' + R_baro;

% Check for reasonable innovation
if abs(y) < params.innovation_gate_baro % OPTIMIZED: Use tuned barometer innovation threshold
    % Kalman gain
    K = P * H_baro' / S;
    
    % Update
    x_est = x_est + K * y;
    P = (eye(9) - K * H_baro) * P;
else
    warning('Barometer measurement rejected due to large innovation: %.1f m', abs(y));
end

end

function [x_est, P] = mag_measurement_update(x_est, P, mag_meas, params)
% Magnetometer measurement update for yaw

% Check for valid magnetometer data first
if isnan(mag_meas)
    % Magnetometer not available - skip update
    return;
end

H_mag = [zeros(1,8), 1]; % Mag measures yaw angle
R_mag = params.R_mag;

% Innovation with angle wrapping
y = wrapToPi(mag_meas - H_mag * x_est);

% Innovation covariance
S = H_mag * P * H_mag' + R_mag;

% Check for reasonable innovation
if abs(y) < params.innovation_gate_mag % OPTIMIZED: Use tuned magnetometer innovation threshold
    % Kalman gain
    K = P * H_mag' / S;
    
    % Update
    x_est = x_est + K * y;
    P = (eye(9) - K * H_mag) * P;
else
    warning('Magnetometer measurement rejected due to large innovation: %.1f°', rad2deg(abs(y)));
end

end

%% UTILITY FUNCTIONS
function x = enforce_state_bounds(x)
% Enforce reasonable bounds on state variables

% Position bounds (±1000m)
x(1:3) = max(min(x(1:3), 1000), -1000);

% Velocity bounds (±100 m/s)
x(4:6) = max(min(x(4:6), 100), -100);

% Attitude bounds (±π/2 for roll/pitch, ±π for yaw)
x(7:8) = max(min(x(7:8), pi/2), -pi/2);
x(9) = wrapToPi(x(9));

end

function P = ensure_positive_definite(P)
% Ensure covariance matrix remains positive definite

try
    % SVD approach for numerical stability
    [U, S, V] = svd(P);
    S = max(S, 1e-12); % Minimum eigenvalue
    S = min(S, 1e6);   % Maximum eigenvalue
    P = U * S * V';
    
    % Make symmetric
    P = (P + P') / 2;
catch
    warning('Covariance matrix repair failed. Using identity matrix.');
    P = 0.1 * eye(size(P));
end

end

function omega = enhanced_attitude_kinematics(att, gyro)
% Enhanced attitude kinematics with singularity handling

phi = att(1); theta = att(2);

% Check for gimbal lock
if abs(cos(theta)) < 1e-6
    % Near singularity - use simplified kinematics
    omega = gyro;
else
    % Normal attitude kinematics
    E = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
         0, cos(phi),           -sin(phi);
         0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
    
    if cond(E) < 1e8 % Check condition number
        omega = E * gyro;
    else
        omega = gyro; % Fallback to body rates
    end
end

% Apply rate limits
max_rate = deg2rad(300); % 300 deg/s maximum
omega = max(min(omega, max_rate), -max_rate);

end

function R = rotation_matrix(phi, theta, psi)
% Rotation matrix from body to NED frame

Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];

R = Rz * Ry * Rx;

end

function [dR_dphi, dR_dtheta, dR_dpsi] = rotation_matrix_derivatives(phi, theta, psi)
% Compute derivatives of rotation matrix for Jacobian

% Derivative with respect to roll (phi)
dRx_dphi = [0 0 0; 0 -sin(phi) -cos(phi); 0 cos(phi) -sin(phi)];
Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
dR_dphi = Rz * Ry * dRx_dphi;

% Derivative with respect to pitch (theta)
Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
dRy_dtheta = [-sin(theta) 0 cos(theta); 0 0 0; -cos(theta) 0 -sin(theta)];
dR_dtheta = Rz * dRy_dtheta * Rx;

% Derivative with respect to yaw (psi)
dRz_dpsi = [-sin(psi) -cos(psi) 0; cos(psi) -sin(psi) 0; 0 0 0];
dR_dpsi = dRz_dpsi * Ry * Rx;

end

function E = euler_rate_matrix(phi, theta)
% Euler rate transformation matrix

if abs(cos(theta)) < 1e-6
    E = eye(3); % Singularity handling
else
    E = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
         0, cos(phi),           -sin(phi);
         0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
end

end

function att = estimate_attitude_from_accel(accel)
% Estimate roll and pitch from accelerometer (assumes stationary)

% Normalize accelerometer reading
accel_norm = accel / norm(accel);

% Calculate roll and pitch
roll = atan2(accel_norm(2), accel_norm(3));
pitch = atan2(-accel_norm(1), sqrt(accel_norm(2)^2 + accel_norm(3)^2));

att = [roll; pitch];

end

function [x_est, P, zupt_applied] = zero_velocity_update(x_est, P, sensors, params)
% IMPROVED: Zero Velocity Update (ZUPT) for enhanced robustness
% Detects when the drone is stationary and applies velocity constraint

zupt_applied = false;

% Extract current velocity estimate
vel_est = x_est(4:6);
vel_magnitude = norm(vel_est);

% Extract IMU measurements for stationary detection
accel = sensors.accel;
gyro = sensors.gyro;

% Remove gravity from accelerometer (simplified)
accel_magnitude = norm(accel - [0; 0; -9.81]);
gyro_magnitude = norm(gyro);

% Thresholds for stationary detection
vel_threshold = 0.2;      % 0.2 m/s velocity threshold
accel_threshold = 0.5;    % 0.5 m/s² acceleration threshold  
gyro_threshold = deg2rad(5); % 5 deg/s angular rate threshold

% Check if drone appears to be stationary
is_stationary = (vel_magnitude < vel_threshold) && ...
                (accel_magnitude < accel_threshold) && ...
                (gyro_magnitude < gyro_threshold);

if is_stationary
    % Apply zero velocity constraint
    H_zupt = [zeros(3,3), eye(3), zeros(3,3)]; % Measure velocity
    R_zupt = diag([0.01, 0.01, 0.01]); % Small measurement noise (1 cm/s)
    
    % Innovation (should be close to zero if stationary)
    y = [0; 0; 0] - H_zupt * x_est;
    
    % Innovation covariance
    S = H_zupt * P * H_zupt' + R_zupt;
    
    % Apply update only if innovation is reasonable
    if norm(y) < 1.0 % Only if velocity error < 1 m/s
        % Kalman gain
        K = P * H_zupt' / S;
        
        % State and covariance update
        x_est = x_est + K * y;
        P = (eye(9) - K * H_zupt) * P;
        
        zupt_applied = true;
    end
end

end