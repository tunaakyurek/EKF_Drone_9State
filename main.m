%% main.m - EKF Drone 9-State Simulation
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
Kp = [0.015; 0.015; 0.022];   % Position proportional gains [x;y;z] (increased)
Ki = [0.0004; 0.0004; 0.0006]; % Integral gains (increased)
Kd = [0.035; 0.035; 0.04];   % Derivative gains (increased)
Kp_yaw = 0.2; Ki_yaw = 0.002; Kd_yaw = 0.04; % (increased)

% PID state
int_err = zeros(3,1); % Integral of position error
prev_err = zeros(3,1); % Previous position error
int_err_yaw = 0; prev_err_yaw = 0;

%% Attitude controller gains (PD)
Kp_att = [1.2; 1.2; 0.8]; % [roll; pitch; yaw] (increased)
Kd_att = [0.4; 0.4; 0.2];

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
    speed_scale = min(1, dist_to_wp / 10); % Slow down when closer than 10m
    % Waypoint switching: simple distance for first segment, robust for others
    if wp_idx == 1
        % For the first segment, use distance check
        if dist_to_wp < 2 && wp_idx < num_wp
            wp_idx = wp_idx + 1;
            wp = waypoints(:,wp_idx);
            err = wp - pos;
            int_err = zeros(3,1);
            prev_err = zeros(3,1);
            int_err_yaw = 0; prev_err_yaw = 0;
            prev_att_err = zeros(3,1);
        end
    else
        % For subsequent segments, use robust switching
        prev_wp = waypoints(:,wp_idx-1);
        wp_vec = wp - prev_wp;
        pos_vec = pos - prev_wp;
        if dot(wp_vec, pos_vec) / norm(wp_vec) > norm(wp_vec) - 2 && wp_idx < num_wp
            wp_idx = wp_idx + 1;
            wp = waypoints(:,wp_idx);
            err = wp - pos;
            int_err = zeros(3,1);
            prev_err = zeros(3,1);
            int_err_yaw = 0; prev_err_yaw = 0;
            prev_att_err = zeros(3,1);
        end
    end
    % Yaw command: last segment (descent) rotates, else face path direction
    if wp_idx == num_wp
        yaw_cmd = mod(yaw + 0.05, 2*pi); % rotate during descent
    else
        yaw_cmd = atan2(err(2), err(1));
    end
    % PID for position (x, y, z) with speed scaling
    int_err = int_err + err*dt;
    derr = (err - prev_err)/dt;
    prev_err = err;
    acc_cmd = speed_scale * (Kp.*err + Ki.*int_err + Kd.*derr);
    % Add velocity damping for smooth stops
    vel_damping = -0.2 * vel; % Damping gain (tune as needed)
    acc_cmd = acc_cmd + vel_damping;
    % Gravity compensation for z
    acc_cmd(3) = acc_cmd(3) + abs(params.g(3));
    % Desired roll and pitch from desired accelerations (assuming small angles)
    des_phi = (1/abs(params.g(3))) * (acc_cmd(1)*sin(yaw) - acc_cmd(2)*cos(yaw));
    des_theta = (1/abs(params.g(3))) * (acc_cmd(1)*cos(yaw) + acc_cmd(2)*sin(yaw));
    max_angle = deg2rad(30); % 30 degrees in radians
    des_phi = max(min(des_phi, max_angle), -max_angle);
    des_theta = max(min(des_theta, max_angle), -max_angle);
    des_att = [des_phi; des_theta; yaw_cmd];
    % Attitude error
    att_err = des_att - att;
    att_err(3) = wrapToPi(att_err(3));
    dattdt = (att_err - prev_att_err)/dt;
    prev_att_err = att_err;
    % PD attitude controller for torques
    tau = Kp_att.*att_err + Kd_att.*dattdt;
    % Saturate torques
    max_tau = 0.2; % [Nm] (typical for small drones)
    tau = max(min(tau, max_tau), -max_tau);
    % Thrust
    thrust = params.mass * acc_cmd(3);
    % Saturate thrust
    min_thrust = 0.1; % [N]
    max_thrust = 2 * params.mass * abs(params.g(3)); % [N]
    thrust = min(max(thrust, min_thrust), max_thrust);
    % Control input: [thrust; tau_phi; tau_theta; tau_psi]
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
        disp('att:'); disp(att');
        error('Simulation diverged: NaN or Inf detected in x_true or u.');
    end
    
    % Generate sensor measurements
    sensors = sensor_model(x_true, params, t(k));
    
    % EKF predict (IMU rate)
    if mod(k-1, round(params.Ts.IMU/dt)) == 0
        [x_est, P] = ekf(x_est, P, u, [], params, params.Ts.IMU, 'IMU');
    end
    % EKF update (GPS)
    if mod(k-1, round(params.Ts.GPS/dt)) == 0
        [x_est, P] = ekf(x_est, P, u, sensors.gps, params, params.Ts.GPS, 'GPS');
        gps_hist(:,k) = sensors.gps;
    end
    % EKF update (Baro)
    if mod(k-1, round(params.Ts.Baro/dt)) == 0
        [x_est, P] = ekf(x_est, P, u, sensors.baro, params, params.Ts.Baro, 'Baro');
    end
    % EKF update (Mag)
    if mod(k-1, round(params.Ts.Mag/dt)) == 0
        [x_est, P] = ekf(x_est, P, u, sensors.mag, params, params.Ts.Mag, 'Mag');
    end
    x_est_hist(:,k) = x_est;
end

%% Animate drone path
animate_drone(t, x_true_hist, x_est_hist, gps_hist, waypoints);

%% Diagnostics: Check for NaN/Inf in simulation data
if any(~isfinite(x_true_hist(:)))
    disp('ERROR: x_true_hist contains NaN or Inf values. Printing first and last 10 positions:');
    disp('First 10 positions:');
    disp(x_true_hist(1:3,1:10));
    disp('Last 10 positions:');
    disp(x_true_hist(1:3,end-9:end));
    figure;
    plot(t, x_true_hist(1,:), 'r', t, x_true_hist(2,:), 'g', t, x_true_hist(3,:), 'b');
    legend('x','y','z');
    title('True Position History (x, y, z)');
    xlabel('Time (s)'); ylabel('Position (m)');
    error('Simulation diverged: NaN or Inf detected in x_true_hist. Check controller and dynamics.');
end

%% Plot results (error plots only after animation)
plot_results(t, x_true_hist, x_est_hist); 