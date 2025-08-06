%% main_stable.m - Stable EKF Drone Simulation with Improved Dynamics
clear; clc; close all;

%% Load parameters
parameters; % Loads 'params' struct

%% Simulation setup
dt = params.Ts.physics;
T_end = params.sim_duration;
t = 0:dt:T_end;
N = length(t);

%% Waypoints (x, y, z) - Same as original
waypoints = [
    0   0   0;
    100 0   0;
    100 100 10;
    0   100 20;
    0   0   30;
    0   0   0
]';
num_wp = size(waypoints,2);
wp_idx = 1;

%% Initial conditions
x0 = zeros(9,1); % [x;y;z;vx;vy;vz;roll;pitch;yaw]
x_true = x0;
x_est = x0;
P = 0.1*eye(9);

x_true_hist = zeros(9,N);
x_est_hist = zeros(9,N);
gps_hist = nan(3,N); % Store GPS measurements (NaN if not sampled)

%% PID controller gains (tuned for smooth, reasonable speed)
Kp = [0.012; 0.012; 0.018];   % Position proportional gains [x;y;z]
Ki = [0.0002; 0.0002; 0.0003]; % Integral gains
Kd = [0.025; 0.025; 0.03];   % Derivative gains
Kp_yaw = 0.15; Ki_yaw = 0.001; Kd_yaw = 0.03;

% PID state
int_err = zeros(3,1); % Integral of position error
prev_err = zeros(3,1); % Previous position error
int_err_yaw = 0; prev_err_yaw = 0;

%% Attitude controller gains (PD) - More conservative
Kp_att = [0.8; 0.8; 0.6]; % [roll; pitch; yaw]
Kd_att = [0.3; 0.3; 0.15];

prev_att_err = zeros(3,1);

%% Main simulation loop
for k = 1:N
    % Waypoint logic
    pos = x_true(1:3);
    vel = x_true(4:6);
    att = x_true(7:9);
    yaw = att(3);
    wp = waypoints(:,wp_idx);
    err = wp - pos;
    dist_to_wp = norm(err);
    
    % FIXED: Calculate yaw command BEFORE waypoint switching
    % This ensures smooth yaw transitions
    if wp_idx == num_wp
        yaw_cmd = mod(yaw + 0.02, 2*pi); % Slower rotation during descent
    else
        % FIXED: Calculate yaw based on NEXT waypoint, not current
        next_wp = waypoints(:,wp_idx);
        if wp_idx < num_wp
            next_wp = waypoints(:,wp_idx + 1);
        end
        path_vec = next_wp - pos;
        yaw_cmd = atan2(path_vec(2), path_vec(1));
    end
    
    % Wrap yaw command to prevent jumps
    yaw_cmd = wrapToPi(yaw_cmd);
    
    % FIXED: Improved speed scaling that considers upcoming turns
    if wp_idx < num_wp
        next_wp = waypoints(:,wp_idx + 1);
        turn_angle = abs(atan2(next_wp(2) - wp(2), next_wp(1) - wp(1)) - atan2(wp(2) - pos(2), wp(1) - pos(1)));
        turn_angle = wrapToPi(turn_angle);
        
        % Reduce speed more aggressively for sharp turns
        if abs(turn_angle) > pi/4 % 45 degrees
            speed_scale = min(1, dist_to_wp / 25); % Slower approach for sharp turns
        else
            speed_scale = min(1, dist_to_wp / 15);
        end
    else
        speed_scale = min(1, dist_to_wp / 15);
    end
    
    % Improved waypoint switching logic
    if wp_idx == 1
        % For the first segment, use distance check
        if dist_to_wp < 5 && wp_idx < num_wp % Increased switching distance
            wp_idx = wp_idx + 1;
            wp = waypoints(:,wp_idx);
            err = wp - pos;
            int_err = zeros(3,1);
            prev_err = zeros(3,1);
            int_err_yaw = 0; prev_err_yaw = 0;
            prev_att_err = zeros(3,1);
            fprintf('Switching to waypoint %d at time %.1fs\n', wp_idx, t(k));
        end
    else
        % For subsequent segments, use more robust switching
        prev_wp = waypoints(:,wp_idx-1);
        wp_vec = wp - prev_wp;
        pos_vec = pos - prev_wp;
        progress = dot(wp_vec, pos_vec) / norm(wp_vec);
        if progress > norm(wp_vec) - 5 && wp_idx < num_wp % Increased switching distance
            wp_idx = wp_idx + 1;
            wp = waypoints(:,wp_idx);
            err = wp - pos;
            int_err = zeros(3,1);
            prev_err = zeros(3,1);
            int_err_yaw = 0; prev_err_yaw = 0;
            prev_att_err = zeros(3,1);
            fprintf('Switching to waypoint %d at time %.1fs\n', wp_idx, t(k));
        end
    end
    
    % PID for position (x, y, z) with improved speed scaling
    int_err = int_err + err*dt;
    derr = (err - prev_err)/dt;
    prev_err = err;
    
    % Add integral windup protection
    int_err = max(min(int_err, 10), -10);
    
    acc_cmd = speed_scale * (Kp.*err + Ki.*int_err + Kd.*derr);
    
    % Improved velocity damping
    vel_damping = -0.15 * vel; % Reduced damping gain
    acc_cmd = acc_cmd + vel_damping;
    
    % Gravity compensation for z
    acc_cmd(3) = acc_cmd(3) + abs(params.g(3));
    
    % Improved desired roll and pitch calculation
    des_phi = (1/abs(params.g(3))) * (acc_cmd(1)*sin(yaw) - acc_cmd(2)*cos(yaw));
    des_theta = (1/abs(params.g(3))) * (acc_cmd(1)*cos(yaw) + acc_cmd(2)*sin(yaw));
    max_angle = deg2rad(20); % Reduced max angle for stability
    des_phi = max(min(des_phi, max_angle), -max_angle);
    des_theta = max(min(des_theta, max_angle), -max_angle);
    des_att = [des_phi; des_theta; yaw_cmd];
    
    % Attitude error with improved wrapping
    att_err = des_att - att;
    att_err(3) = wrapToPi(att_err(3));
    dattdt = (att_err - prev_att_err)/dt;
    prev_att_err = att_err;
    
    % PD attitude controller for torques with saturation
    tau = Kp_att.*att_err + Kd_att.*dattdt;
    
    % More conservative torque limits
    max_tau = 0.15; % [Nm] (reduced)
    tau = max(min(tau, max_tau), -max_tau);
    
    % Improved thrust calculation
    thrust = params.mass * acc_cmd(3);
    
    % More conservative thrust limits
    min_thrust = 0.2; % [N] (increased)
    max_thrust = 1.5 * params.mass * abs(params.g(3)); % [N] (reduced)
    thrust = min(max(thrust, min_thrust), max_thrust);
    
    % Control input: [thrust; tau_phi; tau_theta; tau_psi]
    u = [thrust; tau(1); tau(2); tau(3)];

    % Simulate true drone dynamics with stable model
    x_dot = drone_dynamics_stable(t(k), x_true, u, params);
    x_true = x_true + x_dot*dt;
    x_true_hist(:,k) = x_true;
    
    % In-loop NaN/Inf check
    if any(~isfinite(x_true)) || any(~isfinite(u))
        disp(['NaN/Inf detected at step ', num2str(k)]);
        disp('x_true:'); disp(x_true');
        disp('u:'); disp(u');
        error('Simulation diverged: NaN or Inf detected in x_true or u.');
    end
    
    % Generate sensor measurements
    sensors = sensor_model(x_true, params, t(k));
    imu_meas = [sensors.accel; sensors.gyro]; % [accel; gyro] for EKF
    
    % EKF predict (IMU rate) - Use sensor-only EKF
    if mod(k-1, round(params.Ts.IMU/dt)) == 0
        [x_est, P] = ekf_sensor_only(x_est, P, imu_meas, [], params, params.Ts.IMU, 'IMU');
    end
    
    % EKF update (GPS)
    if mod(k-1, round(params.Ts.GPS/dt)) == 0
        [x_est, P] = ekf_sensor_only(x_est, P, imu_meas, sensors.gps, params, params.Ts.GPS, 'GPS');
        gps_hist(:,k) = sensors.gps;
    end
    
    % EKF update (Baro)
    if mod(k-1, round(params.Ts.Baro/dt)) == 0
        [x_est, P] = ekf_sensor_only(x_est, P, imu_meas, sensors.baro, params, params.Ts.Baro, 'Baro');
    end
    
    % EKF update (Mag)
    if mod(k-1, round(params.Ts.Mag/dt)) == 0
        [x_est, P] = ekf_sensor_only(x_est, P, imu_meas, sensors.mag, params, params.Ts.Mag, 'Mag');
    end
    
    x_est_hist(:,k) = x_est;
end

%% Animate drone path
animate_drone(t, x_true_hist, x_est_hist, gps_hist, waypoints);

%% Plot results with turn analysis
plot_results_improved(t, x_true_hist, x_est_hist, waypoints);

fprintf('\n=== Simulation Complete ===\n');
fprintf('Using sensor-only EKF (no thrust/torque inputs)\n');
fprintf('Using improved dynamics model with cross-coupling effects\n');
fprintf('Using enhanced Jacobian calculation\n');
fprintf('Using improved rotation matrix with numerical stability\n');
fprintf('Using improved controller with better stability\n');
fprintf('FIXED: Yaw command logic and waypoint switching\n'); 