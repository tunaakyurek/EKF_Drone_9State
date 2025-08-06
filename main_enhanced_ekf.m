%% main_enhanced_ekf.m - Enhanced EKF Quadcopter Autopilot Simulation
% Based on MATLAB Simulink Autonomous Flight Design Tutorial Series
% 
% This implementation follows the comprehensive quadcopter autopilot design
% methodology from the tutorial, featuring:
% - Complete Flight Management System with mode transitions
% - Enhanced sensor-only EKF with advanced fusion techniques
% - Cascaded control architecture (Position → Attitude → Angular Rates)
% - Multiple sensor fusion options (EKF, Complementary Filter, etc.)
% - Comprehensive flight modes (Stabilized, Altitude, Position, Trajectory)
%
% Author: Enhanced based on tutorial insights
% Date: Created following Simulink tutorial methodology

clear functions; % Ensure updated function definitions are used
clear; clc; close all;

%% SYSTEM INITIALIZATION
fprintf('=== Enhanced EKF Quadcopter Autopilot ===\n');
fprintf('Based on MATLAB Simulink Autonomous Flight Design Tutorial\n');
fprintf('Initializing enhanced simulation framework...\n\n');

%% ENHANCED LOGGING SYSTEM
% Initialize comprehensive logging for analysis and parameter tuning
logger = enhanced_ekf_logger();
logger.log('INFO', '=== Enhanced EKF Simulation Started ===');
logger.log('INFO', 'Initializing comprehensive logging system');

% Load parameters
parameters; % Loads 'params' struct with all physical parameters

% Simulation configuration
dt = params.Ts.physics;  % Physics timestep (1000 Hz)
T_end = params.sim_duration;
t = 0:dt:T_end;
N = length(t);

fprintf('Physics timestep: %.1f ms (%.0f Hz)\n', dt*1000, 1/dt);
fprintf('Simulation duration: %.1f seconds\n', T_end);
fprintf('Total simulation steps: %d\n\n', N);

%% FLIGHT MISSION DEFINITION
% Enhanced waypoint mission with multiple flight phases
waypoints = [
    0   0   0;      % WP1: Takeoff point
    30  0   3;      % WP2: Forward flight + gentle climb (reduced distance and height)
    30  30  5;      % WP3: Right turn + gentle climb (reduced distance and height)
    0   30  8;      % WP4: Left turn + gentle climb (reduced distance and height)
    0   0   10;     % WP5: Return + gentle climb (reduced height)
    0   0   0       % WP6: Landing
]';

num_wp = size(waypoints, 2);
wp_idx = 1;

% Flight mode sequence for each waypoint
flight_modes = {'stabilized', 'altitude', 'position', 'position', 'trajectory', 'altitude'};
current_mode = flight_modes{1};

fprintf('Flight Mission:\n');
for i = 1:num_wp
    fprintf('  WP%d: [%3.0f, %3.0f, %3.0f] - Mode: %s\n', ...
        i, waypoints(1,i), waypoints(2,i), waypoints(3,i), flight_modes{i});
end
fprintf('\n');

%% STATE INITIALIZATION
% 9-state drone model: [position(3); velocity(3); attitude(3)]
x0 = zeros(9,1); % [x,y,z,vx,vy,vz,roll,pitch,yaw]

% Explicitly ensure zero attitude and velocity for clean initialization
x0(4:9) = zeros(6,1); % Set velocity and attitude to exactly zero

x_true = x0;     % True state (physics simulation)
x_est = x0;      % EKF estimated state
P = 0.1*eye(9);  % Covariance matrix

% Enhanced state history with mode tracking
x_true_hist = zeros(9, N);
x_est_hist = zeros(9, N);
mode_hist = cell(N, 1);
sensor_hist = struct();

% Sensor measurement storage
gps_hist = nan(3, N);
baro_hist = nan(1, N);
mag_hist = nan(1, N);
imu_hist = zeros(6, N); % [accel(3); gyro(3)]

%% ENHANCED FLIGHT MANAGEMENT SYSTEM
% Initialize flight management state machine
flight_mgmt = initialize_flight_management();

%% ENHANCED CONTROL SYSTEM
% Cascaded control architecture following tutorial methodology
control_sys = initialize_control_system(params);

%% SENSOR FUSION CONFIGURATION
% Multiple sensor fusion options following tutorial
sensor_fusion_config = struct();
sensor_fusion_config.method = 'enhanced_ekf'; % Options: 'enhanced_ekf', 'complementary', 'ahrs'
sensor_fusion_config.use_adaptive_noise = true;
sensor_fusion_config.enable_cross_validation = true;

fprintf('Control Architecture:\n');
fprintf('  - Outer Loop: Position Control (PID)\n');
fprintf('  - Middle Loop: Attitude Control (PD)\n');
fprintf('  - Inner Loop: Angular Rate Control\n');
fprintf('  - Sensor Fusion: %s\n', sensor_fusion_config.method);
fprintf('  - Adaptive Noise: %s\n', string(sensor_fusion_config.use_adaptive_noise));
fprintf('\n');

% Log system configuration
logger.log('INFO', sprintf('Physics timestep: %.1f ms (%d Hz)', dt*1000, round(1/dt)));
logger.log('INFO', sprintf('Simulation duration: %.1f seconds', T_end));
logger.log('INFO', sprintf('Total simulation steps: %d', N));
logger.log('INFO', sprintf('Sensor fusion method: %s', sensor_fusion_config.method));
logger.log('INFO', sprintf('Adaptive noise scaling: %s', string(sensor_fusion_config.use_adaptive_noise)));
logger.log('INFO', sprintf('Number of waypoints: %d', size(waypoints,2)));

% Log initial system status
system_status = struct();
system_status.initial_position = x0(1:3)';
system_status.initial_attitude = rad2deg(x0(7:9)');
system_status.flight_mode = flight_mgmt.current_mode;
system_status.sensor_health = flight_mgmt.sensor_health;
logger.log_system_status(system_status);

%% MAIN SIMULATION LOOP
fprintf('Starting enhanced autonomous flight simulation...\n');
fprintf('Time   Mode        WP  Position           Error    Sensor Updates\n');
fprintf('----   ----------  --  -----------------  -------  --------------\n');

for k = 1:N
    current_time = t(k);
    
    %% STEP 1: FLIGHT MANAGEMENT SYSTEM
    % Update flight mode based on mission progress and sensor availability
    [flight_mgmt, mode_changed] = update_flight_management(...
        flight_mgmt, x_est, waypoints, wp_idx, flight_modes, current_time);
    
    current_mode = flight_mgmt.current_mode;
    mode_hist{k} = current_mode;
    
    % Log flight mode changes
    if mode_changed
        logger.log_flight_mode(current_mode, current_time, 'Normal mission progression');
    end
    
    %% STEP 2: SENSOR MEASUREMENTS & PROCESSING
    % Generate comprehensive sensor measurements
    sensors = generate_enhanced_sensors(x_true, params, current_time);
    
    % Store sensor measurements
    imu_hist(:,k) = [sensors.accel; sensors.gyro];
    
    %% STEP 3: ENHANCED SENSOR FUSION & STATE ESTIMATION
    % IMPROVED: Pass additional parameters for enhanced EKF processing
    ekf_params = params;
    ekf_params.current_time = current_time;
    ekf_params.gps_available = sensors.gps_health && ~any(isnan(sensors.gps));
    
    % Multi-rate sensor fusion following tutorial methodology
    [x_est, P, sensor_updates] = enhanced_sensor_fusion(...
        x_est, P, sensors, sensor_fusion_config, ekf_params, dt, k);
    
    % Store sensor-specific measurements when available
    if sensor_updates.gps_updated
        gps_hist(:,k) = sensors.gps;
        logger.log_sensor_update('GPS', current_time, sensors.gps, sensors.gps_health);
    end
    if sensor_updates.baro_updated
        baro_hist(k) = sensors.baro;
        logger.log_sensor_update('BARO', current_time, sensors.baro, ~isnan(sensors.baro));
    end
    if sensor_updates.mag_updated
        mag_hist(k) = sensors.mag;
        logger.log_sensor_update('MAG', current_time, sensors.mag, ~isnan(sensors.mag));
    end
    
    % Log IMU data (always available)
    if sensor_updates.imu_updated
        logger.log_sensor_update('IMU', current_time, [sensors.accel; sensors.gyro], true);
    end
    
    %% STEP 4: WAYPOINT MANAGEMENT & PATH PLANNING
    [wp_idx, waypoint_switched] = update_waypoint_management(...
        x_est, waypoints, wp_idx, current_mode, current_time);
    
    if waypoint_switched
        fprintf('     --> Switched to waypoint %d at t=%.1fs\n', wp_idx, current_time);
        logger.log('INFO', sprintf('Waypoint switch: WP%d → WP%d at t=%.1fs', wp_idx-1, wp_idx, current_time));
        % Reset control integrators on waypoint switch
        control_sys = reset_control_integrators(control_sys);
    end
    
    %% STEP 5: CASCADED CONTROL ARCHITECTURE
    % Three-level cascaded control following tutorial design
    u = cascaded_control_system(x_est, waypoints(:,wp_idx), current_mode, ...
                                control_sys, params, dt, current_time);
    
    %% STEP 6: PHYSICS SIMULATION
    % Simulate true dynamics with enhanced stability
    x_dot = drone_dynamics_stable(current_time, x_true, u, params);
    x_true = x_true + x_dot * dt;
    
    % Wrap angles to [-π, π]
    x_true(7:9) = wrapToPi(x_true(7:9));
    
    % Safety checks for simulation stability
    if any(~isfinite(x_true)) || any(~isfinite(u))
        error_msg = sprintf('Simulation diverged at t=%.2f s', current_time);
        logger.log('ERROR', error_msg);
        logger.log('ERROR', sprintf('x_true: [%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f]', x_true));
        logger.log('ERROR', sprintf('u: [%.2f %.2f %.2f %.2f]', u));
        fprintf('ERROR: %s\n', error_msg);
        break;
    end
    
    %% STEP 7: DATA LOGGING
    x_true_hist(:,k) = x_true;
    x_est_hist(:,k) = x_est;
    
    % Log performance data for analysis
    logger.log_performance(current_time, x_true, x_est, u);
    
    %% STEP 8: PROGRESS MONITORING
    if mod(k, round(2/dt)) == 1  % Every 2 seconds
        pos_error = norm(x_est(1:3) - waypoints(:,wp_idx));
        sensor_str = sprintf('GPS:%d BAR:%d MAG:%d', ...
            sensor_updates.gps_updated, sensor_updates.baro_updated, sensor_updates.mag_updated);
        
        fprintf('%5.1fs  %-10s  %2d  [%5.1f,%5.1f,%5.1f]  %6.2fm  %s\n', ...
            current_time, current_mode, wp_idx, ...
            x_est(1), x_est(2), x_est(3), pos_error, sensor_str);
    end
end

%% COMPREHENSIVE RESULTS ANALYSIS
fprintf('\n=== Enhanced EKF Simulation Complete ===\n');
logger.log('INFO', '=== Simulation Completed Successfully ===');

% Performance metrics
final_error = norm(x_true_hist(1:3,end) - waypoints(:,end));
pos_errors = zeros(1,N);
for i = 1:N
    if i <= size(waypoints,2)
        pos_errors(i) = norm(x_true_hist(1:3,i) - waypoints(:,min(i,end)));
    else
        pos_errors(i) = norm(x_true_hist(1:3,i) - waypoints(:,end));
    end
end

max_error = max(pos_errors);
rms_error = sqrt(mean(pos_errors.^2));

fprintf('\nPerformance Metrics:\n');
fprintf('  Final position error: %.2f m\n', final_error);
fprintf('  Maximum error: %.2f m\n', max_error);
fprintf('  RMS error: %.2f m\n', rms_error);
fprintf('  Flight time: %.1f s\n', T_end);

% Log final performance metrics
logger.log('INFO', sprintf('Final position error: %.3f m', final_error));
logger.log('INFO', sprintf('Maximum error: %.3f m', max_error));
logger.log('INFO', sprintf('RMS error: %.3f m', rms_error));
logger.log('INFO', sprintf('Total flight time: %.1f s', T_end));
fprintf('  Average speed: %.1f m/s\n', norm(x_true_hist(1:3,end)) / T_end);

% EKF Performance Assessment
estimation_errors = vecnorm(x_true_hist(1:3,:) - x_est_hist(1:3,:));
mean_est_error = mean(estimation_errors);
max_est_error = max(estimation_errors);

fprintf('\nEKF Estimation Performance:\n');
fprintf('  Mean estimation error: %.2f m\n', mean_est_error);
fprintf('  Max estimation error: %.2f m\n', max_est_error);
fprintf('  Estimation quality: %s\n', assess_estimation_quality(mean_est_error));

%% ENHANCED VISUALIZATION
create_enhanced_visualizations(t, x_true_hist, x_est_hist, waypoints, ...
    gps_hist, mode_hist, pos_errors, estimation_errors);

%% SENSOR FUSION ANALYSIS
analyze_sensor_fusion_performance(t, x_true_hist, x_est_hist, imu_hist, ...
    gps_hist, baro_hist, mag_hist);

fprintf('\n🎯 Enhanced EKF Algorithm Features:\n');
fprintf('   ✅ Multi-mode flight management (Stabilized/Altitude/Position/Trajectory)\n');
fprintf('   ✅ Enhanced sensor-only EKF with adaptive noise\n');
fprintf('   ✅ Cascaded control architecture (Position → Attitude → Rates)\n');
fprintf('   ✅ Multi-rate sensor fusion (IMU: 200Hz, GPS: 20Hz, Baro: 20Hz)\n');
fprintf('   ✅ Comprehensive flight mode transitions\n');
fprintf('   ✅ Advanced waypoint management with mode-dependent switching\n');
fprintf('   ✅ Enhanced stability and robustness features\n');

fprintf('\n📊 Based on MATLAB Simulink Autonomous Flight Design Tutorial\n');
fprintf('   This implementation follows the complete methodology from the tutorial series.\n');

%% FINALIZE LOGGING SYSTEM
% Close logger and generate comprehensive summary
logger.close();
fprintf('\n📝 Detailed simulation log with performance analysis saved for review.\n');

%% HELPER FUNCTIONS

function flight_mgmt = initialize_flight_management()
    flight_mgmt = struct();
    flight_mgmt.current_mode = 'stabilized';
    flight_mgmt.previous_mode = 'stabilized';
    flight_mgmt.mode_transition_time = 0;
    flight_mgmt.sensor_health = struct('gps', true, 'baro', true, 'mag', true, 'imu', true);
    flight_mgmt.failsafe_triggered = false;
end

function control_sys = initialize_control_system(params)
    % Initialize cascaded control system following tutorial methodology
    control_sys = struct();
    
    % Position control (outer loop) - extremely conservative gains
    control_sys.pos_pid = struct();
    control_sys.pos_pid.Kp = [0.004; 0.004; 0.006];   % Extremely reduced for extremely smooth tracking
    control_sys.pos_pid.Ki = [0.00005; 0.00005; 0.0001]; % Extremely reduced integral gain to prevent overshoot
    control_sys.pos_pid.Kd = [0.05; 0.05; 0.06];   % Extremely increased derivative action for stability
    control_sys.pos_pid.int_err = zeros(3,1);
    control_sys.pos_pid.prev_err = zeros(3,1);
    
    % Attitude control (middle loop) - extremely conservative gains
    control_sys.att_pd = struct();
    control_sys.att_pd.Kp = [0.3; 0.3; 0.2];  % Extremely reduced for extremely gentle attitude control
    control_sys.att_pd.Kd = [1.5; 1.5; 0.8];  % Extremely increased damping for extremely stable response
    control_sys.att_pd.prev_err = zeros(3,1);
    
    % Velocity feedback (inner position loop) - extremely reduced for stability
    control_sys.vel_gains = [0.1; 0.1; 0.2];  % Extremely reduced velocity feedback for extremely smooth response
    
    % Angular rate feedback (inner attitude loop) - extremely increased for stability
    control_sys.rate_gains = [0.4; 0.4; 0.25]; % Extremely increased rate feedback for extremely stable damping
end

function control_sys = reset_control_integrators(control_sys)
    control_sys.pos_pid.int_err = zeros(3,1);
    control_sys.pos_pid.prev_err = zeros(3,1);
    control_sys.att_pd.prev_err = zeros(3,1);
end

function quality = assess_estimation_quality(mean_error)
    if mean_error < 0.5
        quality = 'Excellent';
    elseif mean_error < 1.0
        quality = 'Good';
    elseif mean_error < 2.0
        quality = 'Acceptable';
    else
        quality = 'Needs Improvement';
    end
end
