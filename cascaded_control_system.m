function u = cascaded_control_system(x_est, waypoint, flight_mode, control_sys, params, dt, current_time)
% cascaded_control_system - Three-level cascaded control architecture
% Based on MATLAB Simulink Autonomous Flight Design Tutorial methodology
%
% Implements the complete cascaded control system:
% LEVEL 1: Position Control (Outer Loop) - Generates desired velocity
% LEVEL 2: Attitude Control (Middle Loop) - Generates desired angular rates  
% LEVEL 3: Angular Rate Control (Inner Loop) - Generates torque commands
%
% Control Architecture:
% Position Error → Position PID → Velocity Command → 
% Velocity Error → Attitude Command → Attitude PD → 
% Angular Rate Command → Rate PD → Torque Command
%
% Flight Modes:
% - Stabilized: Manual attitude control, pilot provides thrust
% - Altitude: Altitude hold + manual attitude control
% - Position: Position hold + altitude hold
% - Trajectory: Full autonomous control following waypoints
%
% Inputs:
%   x_est - Current state estimate [pos(3); vel(3); att(3)]
%   waypoint - Target waypoint [x; y; z]
%   flight_mode - Current flight mode string
%   control_sys - Control system parameters structure
%   params - System parameters
%   dt - Control timestep
%   current_time - Current simulation time (optional)
%
% Outputs:
%   u - Control vector [thrust; tau_roll; tau_pitch; tau_yaw]

% Extract state components
pos = x_est(1:3);    % Position [x; y; z] NED
vel = x_est(4:6);    % Velocity [vx; vy; vz] NED
att = x_est(7:9);    % Attitude [roll; pitch; yaw]

% Initialize control output
u = zeros(4,1); % [thrust; tau_roll; tau_pitch; tau_yaw]

%% FLIGHT MODE SPECIFIC CONTROL LOGIC
switch lower(flight_mode)
    case 'stabilized'
        % Manual control with attitude stabilization only
        u = stabilized_mode_control(x_est, control_sys, params);
        
    case 'altitude'
        % Altitude hold + manual horizontal control
        u = altitude_mode_control(x_est, waypoint, control_sys, params, dt);
        
    case 'position'
        % Position hold + altitude hold
        u = position_mode_control(x_est, waypoint, control_sys, params, dt);
        
    case 'trajectory'
        % Full autonomous trajectory following
        u = trajectory_mode_control(x_est, waypoint, control_sys, params, dt);
        
    case 'return_to_home'
        % Emergency return to home
        home_waypoint = [0; 0; -20]; % Home at origin, 20m altitude
        u = trajectory_mode_control(x_est, home_waypoint, control_sys, params, dt);
        
    case 'emergency_land'
        % Emergency landing
        landing_waypoint = [pos(1); pos(2); 0]; % Land at current horizontal position
        u = emergency_landing_control(x_est, landing_waypoint, control_sys, params, dt);
        
    otherwise
        % Default to stabilized mode for safety
        u = stabilized_mode_control(x_est, control_sys, params);
        warning('Unknown flight mode: %s. Using stabilized mode.', flight_mode);
end

% Apply control limits and safety checks
u = apply_control_limits(u, params);

% Handle the case when current_time is not provided
if nargin < 7
    current_time = 0;
end

u = safety_check_control(u, x_est, params, current_time);

end

%% MODE-SPECIFIC CONTROL FUNCTIONS

function u = stabilized_mode_control(x_est, control_sys, params)
% Stabilized mode: Manual control with attitude stabilization

att = x_est(7:9);

% Simulated pilot input (for autonomous simulation)
% In real implementation, this would come from RC transmitter
pilot_roll = 0.0;    % Manual roll command
pilot_pitch = 0.0;   % Manual pitch command  
pilot_yaw_rate = 0.0; % Manual yaw rate command
pilot_thrust = params.mass * abs(params.g(3)) * 0.8; % 80% hover thrust

% Attitude stabilization (Level 2)
att_ref = [pilot_roll; pilot_pitch; att(3)]; % Maintain current yaw
[tau_att, ~] = attitude_control_loop(x_est, att_ref, control_sys, params);

% Yaw rate control (Level 3)
tau_att(3) = yaw_rate_control(x_est, pilot_yaw_rate, control_sys, params);

u = [pilot_thrust; tau_att];

end

function u = altitude_mode_control(x_est, waypoint, control_sys, params, dt)
% Altitude mode: Altitude hold + manual horizontal control

pos = x_est(1:3);
att = x_est(7:9);

% Simulated pilot input for horizontal control
pilot_roll = 0.0;   % Manual roll command
pilot_pitch = 0.0;  % Manual pitch command
pilot_yaw_rate = 0.0; % Manual yaw rate

% Altitude control (Level 1)
altitude_ref = waypoint(3);
[thrust, vel_z_cmd] = altitude_control_loop(x_est, altitude_ref, control_sys, params, dt);

% Manual attitude commands for horizontal control
att_ref = [pilot_roll; pilot_pitch; att(3)]; % Maintain current yaw

% Attitude control (Level 2)
[tau_att, ~] = attitude_control_loop(x_est, att_ref, control_sys, params);

% Yaw rate control (Level 3)
tau_att(3) = yaw_rate_control(x_est, pilot_yaw_rate, control_sys, params);

u = [thrust; tau_att];

end

function u = position_mode_control(x_est, waypoint, control_sys, params, dt)
% Position mode: Position hold + altitude hold

% Full three-level cascaded control
% Level 1: Position control
[vel_cmd, pos_error] = position_control_loop(x_est, waypoint, control_sys, params, dt);

% Level 2: Velocity to attitude control
[att_ref, thrust] = velocity_to_attitude_control(x_est, vel_cmd, control_sys, params);

% Level 2: Attitude control
[tau_att, att_error] = attitude_control_loop(x_est, att_ref, control_sys, params);

% Enhance yaw control for position mode
desired_yaw = calculate_desired_yaw(x_est, waypoint, vel_cmd);
att_ref(3) = desired_yaw;
[tau_att, ~] = attitude_control_loop(x_est, att_ref, control_sys, params);

u = [thrust; tau_att];

end

function u = trajectory_mode_control(x_est, waypoint, control_sys, params, dt)
% Trajectory mode: Full autonomous control with enhanced path following

% Enhanced position control with trajectory anticipation
[vel_cmd, pos_error] = enhanced_position_control(x_est, waypoint, control_sys, params, dt);

% Velocity control with feedforward
[att_ref, thrust] = enhanced_velocity_control(x_est, vel_cmd, control_sys, params);

% Attitude control with rate feedforward
[tau_att, att_error] = enhanced_attitude_control(x_est, att_ref, control_sys, params);

u = [thrust; tau_att];

end

function u = emergency_landing_control(x_est, landing_waypoint, control_sys, params, dt)
% Emergency landing: Controlled descent with position hold

pos = x_est(1:3);
vel = x_est(4:6);

% Override landing waypoint for emergency descent
emergency_descent_rate = -0.8; % m/s descent rate
landing_waypoint(3) = pos(3) + emergency_descent_rate * dt;

% Use position control but with modified gains for gentle descent
control_sys_emergency = control_sys;
control_sys_emergency.pos_pid.Kp(3) = control_sys.pos_pid.Kp(3) * 0.5; % Gentler altitude control
control_sys_emergency.pos_pid.Kd(3) = control_sys.pos_pid.Kd(3) * 2.0; % More damping

u = position_mode_control(x_est, landing_waypoint, control_sys_emergency, params, dt);

% Limit maximum thrust for gentle landing
max_emergency_thrust = params.mass * abs(params.g(3)) * 1.2;
u(1) = min(u(1), max_emergency_thrust);

end

%% CONTROL LOOP IMPLEMENTATIONS

function [vel_cmd, pos_error] = position_control_loop(x_est, pos_ref, control_sys, params, dt)
% Level 1: Position control loop (Outer loop)

pos = x_est(1:3);
vel = x_est(4:6);

% Position error
pos_error = pos_ref - pos;

% PID position control
control_sys.pos_pid.int_err = control_sys.pos_pid.int_err + pos_error * dt;
pos_error_rate = (pos_error - control_sys.pos_pid.prev_err) / dt;
control_sys.pos_pid.prev_err = pos_error;

% Anti-windup for integral term
max_int = 5.0;
control_sys.pos_pid.int_err = max(min(control_sys.pos_pid.int_err, max_int), -max_int);

% PID output (desired velocity)
vel_cmd = control_sys.pos_pid.Kp .* pos_error + ...
          control_sys.pos_pid.Ki .* control_sys.pos_pid.int_err + ...
          control_sys.pos_pid.Kd .* pos_error_rate;

% Velocity feedback for improved stability
vel_error = vel_cmd - vel;
vel_cmd = vel_cmd + control_sys.vel_gains .* vel_error;

% Limit commanded velocity
max_vel = [8; 8; 4]; % [x; y; z] velocity limits (m/s)
vel_cmd = max(min(vel_cmd, max_vel), -max_vel);

end

function [vel_cmd, pos_error] = enhanced_position_control(x_est, pos_ref, control_sys, params, dt)
% Enhanced position control with trajectory prediction and feedforward

pos = x_est(1:3);
vel = x_est(4:6);

% Basic position control
[vel_cmd_basic, pos_error] = position_control_loop(x_est, pos_ref, control_sys, params, dt);

% Add trajectory feedforward for smooth tracking
trajectory_velocity = estimate_trajectory_velocity(pos, pos_ref, dt);
vel_cmd = vel_cmd_basic + 0.3 * trajectory_velocity; % Feedforward gain

% Dynamic speed scaling based on distance to target
distance_to_target = norm(pos_error);
if distance_to_target > 20
    speed_scale = 1.0; % Full speed when far
elseif distance_to_target > 5
    speed_scale = 0.7; % Reduced speed when approaching
else
    speed_scale = 0.4; % Slow speed when close
end

vel_cmd = vel_cmd * speed_scale;

end

function [att_ref, thrust] = velocity_to_attitude_control(x_est, vel_cmd, control_sys, params)
% Convert desired velocity to attitude reference and thrust

vel = x_est(4:6);
att = x_est(7:9);
yaw = att(3);

% Velocity error
vel_error = vel_cmd - vel;

% Desired acceleration (with gravity compensation)
accel_des = control_sys.vel_gains .* vel_error;
accel_des(3) = accel_des(3) + abs(params.g(3)); % Gravity compensation

% Total thrust magnitude
thrust = params.mass * norm(accel_des);

% Desired attitude from acceleration vector
if thrust > 0.1 * params.mass * abs(params.g(3)) % Avoid division by zero
    % Desired body z-axis (thrust direction)
    z_body_des = accel_des / norm(accel_des);
    
    % Calculate desired roll and pitch
    max_tilt = deg2rad(25); % Maximum tilt angle
    
    % Roll (rotation about x-axis)
    roll_des = asin(max(min(-z_body_des(2), sin(max_tilt)), -sin(max_tilt)));
    
    % Pitch (rotation about y-axis)
    if abs(cos(roll_des)) > 1e-6
        pitch_des = asin(max(min(z_body_des(1) / cos(roll_des), sin(max_tilt)), -sin(max_tilt)));
    else
        pitch_des = 0;
    end
    
    % Yaw (maintain current or point toward velocity)
    if norm(vel_cmd) > 1.0
        yaw_des = atan2(vel_cmd(2), vel_cmd(1)); % Point toward desired velocity
    else
        yaw_des = yaw; % Maintain current yaw when hovering
    end
    
    att_ref = [roll_des; pitch_des; yaw_des];
else
    % Low thrust - maintain level attitude
    att_ref = [0; 0; yaw];
    thrust = 0.1 * params.mass * abs(params.g(3)); % Minimum thrust
end

% Limit thrust
max_thrust = 2.0 * params.mass * abs(params.g(3));
min_thrust = 0.1 * params.mass * abs(params.g(3));
thrust = max(min(thrust, max_thrust), min_thrust);

end

function [att_ref, thrust] = enhanced_velocity_control(x_est, vel_cmd, control_sys, params)
% Enhanced velocity control with improved dynamics compensation

% Basic velocity to attitude conversion
[att_ref_basic, thrust_basic] = velocity_to_attitude_control(x_est, vel_cmd, control_sys, params);

% Add dynamic compensation for improved tracking
vel = x_est(4:6);
vel_error = vel_cmd - vel;

% Enhance thrust calculation with velocity feedback
thrust_feedback = params.mass * control_sys.vel_gains(3) * vel_error(3);
thrust = thrust_basic + thrust_feedback;

% Enhance attitude commands with velocity feedforward
vel_magnitude = norm(vel_cmd(1:2));
if vel_magnitude > 0.5
    % Add slight feedforward for aggressive maneuvering
    feedforward_gain = 0.02;
    att_ref = att_ref_basic;
    att_ref(1) = att_ref(1) + feedforward_gain * vel_cmd(2) / abs(params.g(3)); % Roll feedforward
    att_ref(2) = att_ref(2) + feedforward_gain * vel_cmd(1) / abs(params.g(3)); % Pitch feedforward
else
    att_ref = att_ref_basic;
end

end

function [tau, att_error] = attitude_control_loop(x_est, att_ref, control_sys, params)
% Level 2: Attitude control loop (Middle loop)

att = x_est(7:9);

% Attitude error with proper angle wrapping
att_error = att_ref - att;
att_error(3) = wrapToPi(att_error(3)); % Wrap yaw error

% PD attitude control
att_error_rate = (att_error - control_sys.att_pd.prev_err) / 0.001; % Assume 1ms control rate
control_sys.att_pd.prev_err = att_error;

% PD controller output
tau = control_sys.att_pd.Kp .* att_error + control_sys.att_pd.Kd .* att_error_rate;

% Convert to body-frame torques (simplified)
% In full implementation, this would account for inertia matrix and coupling
tau = diag(params.I) .* tau; % Scale by inertia

end

function [tau, att_error] = enhanced_attitude_control(x_est, att_ref, control_sys, params)
% Enhanced attitude control with rate feedforward and adaptive gains

% Basic attitude control
[tau_basic, att_error] = attitude_control_loop(x_est, att_ref, control_sys, params);

% Add rate feedforward for improved tracking
att_rate_ref = calculate_attitude_rate_reference(att_ref, x_est);
rate_feedforward = diag(params.I) .* att_rate_ref * 0.1; % Small feedforward gain

tau = tau_basic + rate_feedforward;

% Adaptive gains based on flight condition
flight_aggressiveness = assess_flight_aggressiveness(x_est);
adaptive_scale = 1.0 + 0.3 * flight_aggressiveness; % Increase gains for aggressive flight

tau = tau * adaptive_scale;

end

function tau_yaw = yaw_rate_control(x_est, yaw_rate_cmd, control_sys, params)
% Level 3: Yaw rate control (Inner loop)

% Simplified yaw rate control
% In full implementation, this would use actual angular rate measurements
current_yaw_rate = 0; % Assume zero rate for simplified model

yaw_rate_error = yaw_rate_cmd - current_yaw_rate;
tau_yaw = control_sys.rate_gains(3) * params.I(3,3) * yaw_rate_error;

end

%% UTILITY FUNCTIONS

function desired_yaw = calculate_desired_yaw(x_est, waypoint, vel_cmd)
% Calculate desired yaw based on velocity direction or waypoint direction

vel = x_est(4:6);
pos = x_est(1:3);

% Point toward velocity direction when moving
if norm(vel_cmd(1:2)) > 1.0
    desired_yaw = atan2(vel_cmd(2), vel_cmd(1));
elseif norm(waypoint(1:2) - pos(1:2)) > 2.0
    % Point toward waypoint when stationary but far from target
    direction_to_wp = waypoint(1:2) - pos(1:2);
    desired_yaw = atan2(direction_to_wp(2), direction_to_wp(1));
else
    % Maintain current yaw when close to target
    desired_yaw = x_est(9);
end

end

function traj_vel = estimate_trajectory_velocity(current_pos, target_pos, dt)
% Estimate trajectory velocity for feedforward control

persistent prev_target_pos prev_time;

if isempty(prev_target_pos)
    prev_target_pos = target_pos;
    prev_time = 0;
    traj_vel = zeros(3,1);
else
    % Estimate target velocity from target position changes
    target_vel = (target_pos - prev_target_pos) / dt;
    traj_vel = target_vel;
    
    prev_target_pos = target_pos;
    prev_time = prev_time + dt;
end

% Limit trajectory velocity
max_traj_vel = 5.0; % m/s
traj_vel_norm = norm(traj_vel);
if traj_vel_norm > max_traj_vel
    traj_vel = traj_vel * (max_traj_vel / traj_vel_norm);
end

end

function att_rate_ref = calculate_attitude_rate_reference(att_ref, x_est)
% Calculate reference attitude rates for feedforward

persistent prev_att_ref;

if isempty(prev_att_ref)
    prev_att_ref = att_ref;
    att_rate_ref = zeros(3,1);
else
    dt = 0.001; % Assume 1ms control rate
    att_rate_ref = (att_ref - prev_att_ref) / dt;
    prev_att_ref = att_ref;
end

% Limit attitude rates
max_rate = deg2rad(100); % 100 deg/s
att_rate_ref = max(min(att_rate_ref, max_rate), -max_rate);

end

function aggressiveness = assess_flight_aggressiveness(x_est)
% Assess flight aggressiveness for adaptive control gains

vel = x_est(4:6);
att = x_est(7:9);

% Velocity-based assessment
vel_aggressiveness = norm(vel) / 10; % Normalize by 10 m/s

% Attitude-based assessment
att_aggressiveness = norm(att(1:2)) / deg2rad(30); % Normalize by 30 degrees

% Combined assessment
aggressiveness = max(vel_aggressiveness, att_aggressiveness);
aggressiveness = max(min(aggressiveness, 1.0), 0.0); % Clamp to [0, 1]

end

function u = apply_control_limits(u, params)
% Apply physical control limits

% Thrust limits
max_thrust = 3.0 * params.mass * abs(params.g(3));
min_thrust = 0.05 * params.mass * abs(params.g(3));
u(1) = max(min(u(1), max_thrust), min_thrust);

% Torque limits
max_torque = 0.2; % Nm
u(2:4) = max(min(u(2:4), max_torque), -max_torque);

end

function u = safety_check_control(u, x_est, params, current_time)
% IMPROVED: Enhanced safety checks with attitude recovery
%
% Inputs:
%   u - Control vector [thrust; tau_roll; tau_pitch; tau_yaw]
%   x_est - Current state estimate [pos(3); vel(3); att(3)]
%   params - System parameters
%   current_time - Current simulation time (used to skip warnings during initialization)

% Handle the case when current_time is not provided
if nargin < 4
    current_time = 1.0; % Default to non-initialization time
end

% Define initialization period (first 1.0 seconds for better stabilization)
is_initialization = (current_time < 1.0);

% Extract state components
att = x_est(7:9);
vel = x_est(4:6);
roll = att(1); pitch = att(2); yaw = att(3);

% ENHANCED ATTITUDE RECOVERY SYSTEM
% Check for severe attitude deviations - adjusted thresholds for stable operation
severe_attitude_threshold = deg2rad(20); % Adjusted threshold for warnings
critical_attitude_threshold = deg2rad(30); % Adjusted critical threshold

attitude_magnitude = norm(att(1:2));

% Always apply some attitude damping for stability
u(2) = u(2) - 0.5 * roll; % Always apply roll damping
u(3) = u(3) - 0.5 * pitch; % Always apply pitch damping

if attitude_magnitude > critical_attitude_threshold
    % CRITICAL: Implement emergency attitude recovery
    if ~is_initialization
        warning('CRITICAL: Severe attitude deviation (%.1f°). Emergency recovery mode.', rad2deg(attitude_magnitude));
    end
    
    % Emergency attitude recovery - extremely aggressive stabilization
    recovery_gain = 10.0; % Extremely increased for immediate recovery
    u(1) = 0.2 * params.mass * abs(params.g(3)); % Minimal thrust for maximum stability
    u(2) = -recovery_gain * sign(roll) * min(abs(roll), deg2rad(15)); % Extremely strong roll recovery
    u(3) = -recovery_gain * sign(pitch) * min(abs(pitch), deg2rad(15)); % Extremely strong pitch recovery
    u(4) = -2.0 * sign(yaw) * min(abs(yaw), deg2rad(30)); % Extremely strong yaw damping
    
    % Force a complete reset of the attitude control for any critical deviation
    % Complete override of control signals for emergency recovery
    u(1) = 0.15 * params.mass * abs(params.g(3)); % Minimal thrust
    u(2) = -15.0 * sign(roll) * min(abs(roll), deg2rad(10)); 
    u(3) = -15.0 * sign(pitch) * min(abs(pitch), deg2rad(10));
    u(4) = -3.0 * sign(yaw) * min(abs(yaw), deg2rad(20));
    
elseif attitude_magnitude > severe_attitude_threshold
    % SEVERE: Reduce control authority and add recovery bias
    if ~is_initialization
        warning('Safety: Severe attitude detected (%.1f°). Reducing control authority.', rad2deg(attitude_magnitude));
    end
    
    % Gradual attitude recovery with extremely strong correction
    recovery_factor = 0.1; % Extremely reduced for more aggressive safety
    u(1) = min(u(1), 0.3 * params.mass * abs(params.g(3))); % Extremely low thrust limit
    u(2) = recovery_factor * u(2) - 3.0 * roll; % Extremely strong roll recovery bias
    u(3) = recovery_factor * u(3) - 3.0 * pitch; % Extremely strong pitch recovery bias
    u(4) = recovery_factor * u(4) - 1.0 * sign(yaw) * min(abs(yaw), deg2rad(30)); % Extremely strong yaw damping
    
elseif attitude_magnitude > deg2rad(12) % Adjusted threshold for moderate corrections
    % MODERATE: Apply safety reduction for moderate attitudes
    u(1) = min(u(1), 0.7 * params.mass * abs(params.g(3)));
    u(2:4) = u(2:4) * 0.5; % Moderate reduction for safety
end

% Check for excessive velocities - adjusted for normal operation
if norm(vel) > 10 % Adjusted threshold for normal flight speeds
    % Calculate velocity reduction factor based on speed - extremely aggressive reduction
    vel_mag = norm(vel);
    vel_reduction = max(0.2, 1.0 - (vel_mag - 3) / 2); % Extremely aggressive progressive reduction
    
    % Apply extremely strong limiting for high velocities
    u(1) = min(u(1), vel_reduction * params.mass * abs(params.g(3)));
    
    % Apply extremely strong corrective torques to slow down
    vel_dir = vel(1:2) / norm(vel(1:2) + 1e-6);
    u(2) = u(2) - 0.3 * vel_dir(2) * vel_mag; % Extremely strong roll correction
    u(3) = u(3) + 0.3 * vel_dir(1) * vel_mag; % Extremely strong pitch correction
    
    % Apply additional yaw correction to align with velocity direction
    desired_yaw = atan2(vel(2), vel(1)); % Always align with velocity
    yaw_error = wrapToPi(desired_yaw - yaw);
    u(4) = u(4) + 0.2 * yaw_error; % Stronger yaw correction
    
    % Only show warning if not in initialization phase
    if ~is_initialization
        warning('Safety: High velocity detected (%.1f m/s). Limiting thrust.', vel_mag);
    end
    
    % Emergency braking for extreme velocities
    if vel_mag > 15
        u(1) = 0.1 * params.mass * abs(params.g(3)); % Minimal thrust for emergency braking
        u(2) = -0.5 * vel_dir(2) * vel_mag; % Maximum roll correction
        u(3) = 0.5 * vel_dir(1) * vel_mag; % Maximum pitch correction
    end
end

% Check for angular velocity limits (prevent spinning) - extremely restrictive
if isfield(params, 'max_angular_rate')
    max_rate = params.max_angular_rate;
else
    max_rate = deg2rad(45); % Extremely reduced from 90 deg/s
end

% Limit commanded torques to prevent excessive angular rates - extremely restrictive
max_torque = 0.02; % Extremely reduced from 0.05 N⋅m
u(2:4) = max(min(u(2:4), max_torque), -max_torque);

% Final safety check - ensure thrust is never too high during recovery
if attitude_magnitude > deg2rad(10) || norm(vel) > 5
    max_safe_thrust = 0.5 * params.mass * abs(params.g(3));
    u(1) = min(u(1), max_safe_thrust);
end

end

function [thrust, vel_z_cmd] = altitude_control_loop(x_est, altitude_ref, control_sys, params, dt)
% Altitude control loop for altitude mode
% Level 1 control: Altitude reference to thrust command

pos = x_est(1:3);
vel = x_est(4:6);

% Current altitude (convert from NED down to altitude up)
current_altitude = -pos(3);

% Altitude error
altitude_error = altitude_ref - current_altitude;

% Altitude PID control (simplified - using only altitude component of position gains)
persistent alt_int_err alt_prev_err;
if isempty(alt_int_err)
    alt_int_err = 0;
    alt_prev_err = 0;
end

% Integral with anti-windup
alt_int_err = alt_int_err + altitude_error * dt;
max_int = 5.0; % Anti-windup limit
alt_int_err = max(min(alt_int_err, max_int), -max_int);

% Derivative
alt_error_rate = (altitude_error - alt_prev_err) / dt;
alt_prev_err = altitude_error;

% Altitude PID gains (use Z component of position gains)
Kp_alt = control_sys.pos_pid.Kp(3);
Ki_alt = control_sys.pos_pid.Ki(3);
Kd_alt = control_sys.pos_pid.Kd(3);

% Desired vertical velocity (up positive)
vel_z_cmd = Kp_alt * altitude_error + Ki_alt * alt_int_err + Kd_alt * alt_error_rate;

% Limit commanded vertical velocity
max_vel_z = 4.0; % m/s
vel_z_cmd = max(min(vel_z_cmd, max_vel_z), -max_vel_z);

% Velocity feedback for thrust calculation
current_vel_z = -vel(3); % Convert NED down to up
vel_z_error = vel_z_cmd - current_vel_z;

% Velocity feedback gain
Kp_vel_z = 0.8;
accel_z_cmd = Kp_vel_z * vel_z_error;

% Thrust calculation (including gravity compensation)
thrust = params.mass * (accel_z_cmd + abs(params.g(3)));

% Apply thrust limits
max_thrust = 2.0 * params.mass * abs(params.g(3));
min_thrust = 0.1 * params.mass * abs(params.g(3));
thrust = max(min(thrust, max_thrust), min_thrust);

end