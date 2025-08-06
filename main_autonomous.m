%% main_autonomous.m - Autonomous Drone EKF Simulation with Built-in Control
clear; clc; close all;

%% Load parameters
parameters; % Loads 'params' struct

%% Simulation setup
dt = params.Ts.physics;
T_end = params.sim_duration;
t = 0:dt:T_end;
N = length(t);

%% Waypoints (x, y, z)
waypoints = [
    0   0   0;
    100 0   0;
    100 100 10;
    0   100 20;
    0   0   30;
    0   0   0
]';

%% Generate smooth 3D trajectory using spline interpolation
% Create time parameter for waypoints
num_wp = size(waypoints, 2);
wp_times = linspace(0, T_end, num_wp);

% Generate dense time vector for smooth trajectory
time_ref = 0:0.1:T_end;

% Interpolate 3D trajectory using splines
ref_traj = zeros(3, length(time_ref));
ref_vel = zeros(3, length(time_ref));
ref_yaw = zeros(1, length(time_ref));

for i = 1:length(time_ref)
    t_current = time_ref(i);
    
    % Find current segment
    if t_current >= wp_times(end)
        % At or past last waypoint, stay at last waypoint
        ref_traj(:, i) = waypoints(:, end);
        ref_vel(:, i) = [0; 0; 0];
        ref_yaw(i) = 0;
    else
        % Interpolate between waypoints
        for j = 1:num_wp-1
            if t_current >= wp_times(j) && t_current <= wp_times(j+1)
                % Linear interpolation between waypoints
                alpha = (t_current - wp_times(j)) / (wp_times(j+1) - wp_times(j));
                ref_traj(:, i) = (1-alpha) * waypoints(:, j) + alpha * waypoints(:, j+1);
                
                % Velocity (constant between waypoints)
                segment_vel = (waypoints(:, j+1) - waypoints(:, j)) / (wp_times(j+1) - wp_times(j));
                ref_vel(:, i) = segment_vel;
                
                % Yaw (direction to next waypoint)
                if j < num_wp-1
                    next_wp = waypoints(:, j+1);
                    current_pos = ref_traj(:, i);
                    ref_yaw(i) = atan2(next_wp(2) - current_pos(2), next_wp(1) - current_pos(1));
                else
                    ref_yaw(i) = 0;
                end
                break;
            end
        end
    end
end

%% Initial conditions
x0 = zeros(9,1); % [x;y;z;vx;vy;vz;roll;pitch;yaw]
x_true = x0;
x_est = x0;
P = eye(9);

x_true_hist = zeros(9,N);
x_est_hist = zeros(9,N);
gps_hist = nan(3,N); % Store GPS measurements (NaN if not sampled)

%% Autonomous controller gains (tuned for smooth tracking)
Kp_pos = [2.0; 2.0; 2.5];   % Position tracking gains
Kd_pos = [1.0; 1.0; 1.2];   % Velocity tracking gains
Kp_att = [3.0; 3.0; 2.0];   % Attitude tracking gains
Kd_att = [1.0; 1.0; 0.8];   % Angular rate tracking gains

%% Main simulation loop
for k = 1:N
    current_time = t(k);
    
    % Get reference trajectory at current time
    [~, idx] = min(abs(time_ref - current_time));
    pos_ref = ref_traj(:, idx);
    vel_ref = ref_vel(:, idx);
    yaw_ref = ref_yaw(idx);
    
    % Current state
    pos = x_true(1:3);
    vel = x_true(4:6);
    att = x_true(7:9);
    yaw = att(3);
    
    % Position and velocity errors
    pos_err = pos_ref - pos;
    vel_err = vel_ref - vel;
    
    % Desired acceleration from position/velocity controller
    acc_des = Kp_pos.*pos_err + Kd_pos.*vel_err;
    acc_des(3) = acc_des(3) + abs(params.g(3)); % Gravity compensation
    
    % Convert desired acceleration to desired attitude (small angle approximation)
    des_phi = (1/abs(params.g(3))) * (acc_des(1)*sin(yaw) - acc_des(2)*cos(yaw));
    des_theta = (1/abs(params.g(3))) * (acc_des(1)*cos(yaw) + acc_des(2)*sin(yaw));
    des_att = [des_phi; des_theta; yaw_ref];
    
    % Clamp desired angles
    max_angle = deg2rad(30);
    des_att(1) = max(min(des_att(1), max_angle), -max_angle);
    des_att(2) = max(min(des_att(2), max_angle), -max_angle);
    
    % Attitude error
    att_err = des_att - att;
    att_err(3) = wrapToPi(att_err(3));
    
    % Angular rate error (approximate)
    dattdt = (att_err - [0;0;0]) / dt; % Simple derivative approximation
    
    % Attitude controller for torques
    tau = Kp_att.*att_err + Kd_att.*dattdt;
    
    % Saturate torques
    max_tau = 0.2; % [Nm]
    tau = max(min(tau, max_tau), -max_tau);
    
    % Thrust
    thrust = params.mass * acc_des(3);
    min_thrust = 0.1; % [N]
    max_thrust = 2 * params.mass * abs(params.g(3)); % [N]
    thrust = min(max(thrust, min_thrust), max_thrust);
    
    % Control input (for true dynamics)
    u = [thrust; tau(1); tau(2); tau(3)];
    
    % Simulate true drone dynamics
    x_dot = drone_dynamics(t(k), x_true, u, params);
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
    
    % Always perform EKF prediction at IMU rate using IMU data
    [x_est, P] = ekf(x_est, P, imu_meas, [], params, params.Ts.IMU, 'IMU');
    % Perform measurement updates if available
    if mod(k-1, round(params.Ts.GPS/dt)) == 0
        [x_est, P] = ekf(x_est, P, imu_meas, sensors.gps, params, params.Ts.GPS, 'GPS');
        gps_hist(:,k) = sensors.gps;
    end
    if mod(k-1, round(params.Ts.Baro/dt)) == 0
        [x_est, P] = ekf(x_est, P, imu_meas, sensors.baro, params, params.Ts.Baro, 'Baro');
    end
    if mod(k-1, round(params.Ts.Mag/dt)) == 0
        [x_est, P] = ekf(x_est, P, imu_meas, sensors.mag, params, params.Ts.Mag, 'Mag');
    end
    % Always output the latest prediction
    x_est_hist(:,k) = x_est;
end

%% Animate drone path
animate_drone(t, x_true_hist, x_est_hist, gps_hist, waypoints);

%% Plot results (error plots only after animation)
plot_results(t, x_true_hist, x_est_hist);

%% Plot reference trajectory vs actual
figure;
subplot(3,1,1);
plot(t, x_true_hist(1,:), 'b', 'LineWidth', 2); hold on;
plot(time_ref, ref_traj(1,:), 'r--', 'LineWidth', 2);
ylabel('X (m)'); legend('Actual', 'Reference'); grid on;

subplot(3,1,2);
plot(t, x_true_hist(2,:), 'b', 'LineWidth', 2); hold on;
plot(time_ref, ref_traj(2,:), 'r--', 'LineWidth', 2);
ylabel('Y (m)'); legend('Actual', 'Reference'); grid on;

subplot(3,1,3);
plot(t, x_true_hist(3,:), 'b', 'LineWidth', 2); hold on;
plot(time_ref, ref_traj(3,:), 'r--', 'LineWidth', 2);
ylabel('Z (m)'); xlabel('Time (s)'); legend('Actual', 'Reference'); grid on;

sgtitle('Reference Trajectory vs Actual Path'); 