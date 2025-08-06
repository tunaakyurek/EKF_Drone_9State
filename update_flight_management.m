function [flight_mgmt, mode_changed] = update_flight_management(flight_mgmt, x_est, waypoints, wp_idx, flight_modes, current_time)
% update_flight_management - Comprehensive flight management system
% Based on MATLAB Simulink Autonomous Flight Design Tutorial methodology
%
% Implements the complete flight management system including:
% - Mode transition logic (Stabilized → Altitude → Position → Trajectory)
% - Sensor health monitoring and failsafe logic
% - Mission management and waypoint sequencing
% - Safety checks and emergency procedures
%
% Flight Modes (following tutorial specification):
% - Stabilized: Manual control with attitude stabilization
% - Altitude: Altitude hold + manual horizontal control
% - Position: Position hold + altitude hold
% - Trajectory: Autonomous trajectory following
%
% Inputs:
%   flight_mgmt - Flight management state structure
%   x_est - Current state estimate [pos(3); vel(3); att(3)]
%   waypoints - Mission waypoints [3 x N]
%   wp_idx - Current waypoint index
%   flight_modes - Mode sequence for each waypoint
%   current_time - Current simulation time
%
% Outputs:
%   flight_mgmt - Updated flight management state
%   mode_changed - Boolean indicating if mode transition occurred

mode_changed = false;
previous_mode = flight_mgmt.current_mode;

%% SENSOR HEALTH ASSESSMENT
% Monitor sensor health for mode transition decisions
sensor_health = assess_sensor_health(flight_mgmt, current_time);
flight_mgmt.sensor_health = sensor_health;

%% FLIGHT MODE TRANSITION LOGIC
% Following the tutorial's state machine methodology

% IMPROVED: Add minimum time between mode transitions to prevent rapid switching
MIN_MODE_TIME = 2.0; % Minimum 2 seconds in each mode
if isfield(flight_mgmt, 'mode_transition_time')
    time_in_current_mode = current_time - flight_mgmt.mode_transition_time;
    if time_in_current_mode < MIN_MODE_TIME
        % Too soon to switch modes, skip transition logic
        return;
    end
end

switch flight_mgmt.current_mode
    case 'disarmed'
        % Transition to armed/stabilized when conditions are met
        if is_safe_to_arm(x_est, sensor_health)
            flight_mgmt.current_mode = 'stabilized';
            mode_changed = true;
            fprintf('t=%.1fs: Armed → Stabilized Mode\n', current_time);
        end
        
    case 'stabilized'
        % Transition to altitude mode when altitude sensor is healthy
        if sensor_health.baro && is_stable_flight(x_est)
            if wp_idx <= length(flight_modes) && strcmp(flight_modes{wp_idx}, 'altitude')
                flight_mgmt.current_mode = 'altitude';
                mode_changed = true;
                fprintf('t=%.1fs: Stabilized → Altitude Mode\n', current_time);
            end
        end
        
    case 'altitude'
        % Transition to position mode when GPS is healthy
        if sensor_health.gps && sensor_health.baro
            if wp_idx <= length(flight_modes) && strcmp(flight_modes{wp_idx}, 'position')
                flight_mgmt.current_mode = 'position';
                mode_changed = true;
                fprintf('t=%.1fs: Altitude → Position Mode\n', current_time);
            end
        end
        
        % Failsafe: Revert to stabilized if altitude sensor fails
        if ~sensor_health.baro
            flight_mgmt.current_mode = 'stabilized';
            mode_changed = true;
            fprintf('t=%.1fs: FAILSAFE - Altitude → Stabilized (Baro failure)\n', current_time);
        end
        
    case 'position'
        % Transition to trajectory mode for autonomous flight
        if sensor_health.gps && sensor_health.baro && sensor_health.mag
            if wp_idx <= length(flight_modes) && strcmp(flight_modes{wp_idx}, 'trajectory')
                flight_mgmt.current_mode = 'trajectory';
                mode_changed = true;
                fprintf('t=%.1fs: Position → Trajectory Mode\n', current_time);
            end
        end
        
        % Failsafe: Degrade based on sensor failures
        if ~sensor_health.gps && sensor_health.baro
            flight_mgmt.current_mode = 'altitude';
            mode_changed = true;
            fprintf('t=%.1fs: FAILSAFE - Position → Altitude (GPS failure)\n', current_time);
        elseif ~sensor_health.baro
            flight_mgmt.current_mode = 'stabilized';
            mode_changed = true;
            fprintf('t=%.1fs: FAILSAFE - Position → Stabilized (Baro failure)\n', current_time);
        end
        
    case 'trajectory'
        % Remain in trajectory mode while all sensors are healthy
        % Failsafe: Degrade based on sensor failures
        if ~sensor_health.gps && sensor_health.baro
            flight_mgmt.current_mode = 'position';
            mode_changed = true;
            fprintf('t=%.1fs: FAILSAFE - Trajectory → Position (GPS failure)\n', current_time);
        elseif ~sensor_health.baro && sensor_health.gps
            flight_mgmt.current_mode = 'position';
            mode_changed = true;
            fprintf('t=%.1fs: FAILSAFE - Trajectory → Position (Baro failure)\n', current_time);
        elseif ~sensor_health.gps && ~sensor_health.baro
            flight_mgmt.current_mode = 'stabilized';
            mode_changed = true;
            fprintf('t=%.1fs: FAILSAFE - Trajectory → Stabilized (GPS+Baro failure)\n', current_time);
        end
        
    case 'return_to_home'
        % Emergency return to home mode
        % Only exit when safe to land
        home_distance = norm(x_est(1:3) - [0; 0; 0]); % Assume home at origin
        if home_distance < 2.0 && abs(x_est(6)) < 0.5 % Close to home and low vertical velocity
            flight_mgmt.current_mode = 'stabilized';
            mode_changed = true;
            fprintf('t=%.1fs: Return to Home → Stabilized (Arrived at home)\n', current_time);
        end
        
    case 'emergency_land'
        % Emergency landing mode - stay until landed
        if -x_est(3) < 0.5 && abs(x_est(6)) < 0.2 % Close to ground with low velocity
            flight_mgmt.current_mode = 'disarmed';
            mode_changed = true;
            fprintf('t=%.1fs: Emergency Land → Disarmed (Landed)\n', current_time);
        end
end

%% EMERGENCY CONDITIONS OVERRIDE
% Critical safety checks that override normal mode transitions

% Battery low emergency
if is_battery_low(current_time)
    if ~strcmp(flight_mgmt.current_mode, 'return_to_home') && ...
       ~strcmp(flight_mgmt.current_mode, 'emergency_land')
        flight_mgmt.current_mode = 'return_to_home';
        mode_changed = true;
        fprintf('t=%.1fs: EMERGENCY - Low Battery → Return to Home\n', current_time);
    end
end

% Complete sensor failure
if ~sensor_health.imu
    flight_mgmt.current_mode = 'emergency_land';
    mode_changed = true;
    fprintf('t=%.1fs: CRITICAL - IMU failure → Emergency Land\n', current_time);
end

% Geofence violation
if is_geofence_violation(x_est)
    if ~strcmp(flight_mgmt.current_mode, 'return_to_home')
        flight_mgmt.current_mode = 'return_to_home';
        mode_changed = true;
        fprintf('t=%.1fs: SAFETY - Geofence violation → Return to Home\n', current_time);
    end
end

%% MODE TRANSITION VALIDATION
% Ensure mode transitions are valid and safe
if mode_changed
    % Validate the mode transition
    if ~is_valid_mode_transition(previous_mode, flight_mgmt.current_mode)
        fprintf('Warning: Invalid mode transition %s → %s. Reverting.\n', ...
            previous_mode, flight_mgmt.current_mode);
        flight_mgmt.current_mode = previous_mode;
        mode_changed = false;
    else
        % Update transition timestamp
        flight_mgmt.mode_transition_time = current_time;
        flight_mgmt.previous_mode = previous_mode;
        
        % Mode-specific initialization
        flight_mgmt = initialize_mode_specific_parameters(flight_mgmt, x_est);
    end
end

%% FLIGHT MANAGEMENT STATUS UPDATE
flight_mgmt.system_status = assess_overall_system_status(flight_mgmt, x_est);

end

%% HELPER FUNCTIONS

function sensor_health = assess_sensor_health(flight_mgmt, current_time)
% IMPROVED: Assess health with hysteresis to prevent mode flickering

persistent last_gps_time last_baro_time last_mag_time last_health_state;

% Initialize persistent variables
if isempty(last_gps_time)
    last_gps_time = current_time;
    last_baro_time = current_time;
    last_mag_time = current_time;
    % Initialize with all sensors healthy
    last_health_state = struct('gps', true, 'baro', true, 'mag', true, 'imu', true);
end

% IMPROVED: Timeout thresholds with hysteresis
gps_timeout_fail = 3.0;   % GPS fails after 3 seconds (increased)
gps_timeout_recover = 1.0; % GPS recovers after 1 second of good data
baro_timeout_fail = 2.0;  % Barometer fails after 2 seconds
baro_timeout_recover = 0.5; % Barometer recovers after 0.5 seconds
mag_timeout_fail = 2.0;   % Magnetometer fails after 2 seconds
mag_timeout_recover = 0.5; % Magnetometer recovers after 0.5 seconds
imu_timeout = 0.1;        % IMU should update every 100ms

% Default health status
sensor_health = struct();

% IMU health (rarely fails completely)
sensor_health.imu = true; % Assume healthy unless proven otherwise

% IMPROVED: GPS health assessment with hysteresis
gps_age = current_time - last_gps_time;
if last_health_state.gps
    % GPS was healthy, check if it should fail
    sensor_health.gps = gps_age < gps_timeout_fail;
else
    % GPS was unhealthy, check if it should recover
    sensor_health.gps = gps_age < gps_timeout_recover;
end

% Update GPS timestamp if we received new measurement
if isfield(flight_mgmt, 'last_gps_update')
    if flight_mgmt.last_gps_update > last_gps_time
        last_gps_time = flight_mgmt.last_gps_update;
    end
end

% IMPROVED: Barometer health assessment with hysteresis
baro_age = current_time - last_baro_time;
if last_health_state.baro
    sensor_health.baro = baro_age < baro_timeout_fail;
else
    sensor_health.baro = baro_age < baro_timeout_recover;
end

% IMPROVED: Magnetometer health assessment with hysteresis
mag_age = current_time - last_mag_time;
if last_health_state.mag
    sensor_health.mag = mag_age < mag_timeout_fail;
else
    sensor_health.mag = mag_age < mag_timeout_recover;
end

% Additional health checks based on measurement quality
sensor_health.gps = sensor_health.gps && check_gps_quality(flight_mgmt);
sensor_health.baro = sensor_health.baro && check_baro_quality(flight_mgmt);
sensor_health.mag = sensor_health.mag && check_mag_quality(flight_mgmt);

% IMPROVED: Update health state history for next iteration
last_health_state = sensor_health;

end

function safe = is_safe_to_arm(x_est, sensor_health)
% Check if it's safe to arm the vehicle

% Basic safety checks
altitude_safe = -x_est(3) < 1.0; % Less than 1m altitude
velocity_safe = norm(x_est(4:6)) < 0.5; % Low velocity
attitude_safe = norm(x_est(7:8)) < deg2rad(15); % Not tilted more than 15°
sensors_healthy = sensor_health.imu; % At minimum, IMU must be healthy

safe = altitude_safe && velocity_safe && attitude_safe && sensors_healthy;

end

function stable = is_stable_flight(x_est)
% Check if the vehicle is in stable flight suitable for mode upgrades

velocity_stable = norm(x_est(4:6)) < 2.0; % Velocity < 2 m/s
attitude_stable = norm(x_est(7:8)) < deg2rad(20); % Roll/pitch < 20°

stable = velocity_stable && attitude_stable;

end

function battery_low = is_battery_low(current_time)
% IMPROVED: More realistic battery model with longer flight time

% Simulate battery discharge over time
flight_time = current_time;
battery_capacity = 30 * 60; % 30 minutes flight time (more realistic for demo)
discharge_rate = flight_time / battery_capacity;

% Consider low battery at 10% remaining (much more conservative)
battery_low = discharge_rate > 0.9;

% REMOVED: Random battery failures for more predictable testing
% battery_low = battery_low || (rand() < 0.001); % Commented out

end

function violation = is_geofence_violation(x_est)
% Check for geofence violations

% Define geofence boundaries
max_altitude = 50;    % meters
max_horizontal = 200; % meters

altitude_violation = -x_est(3) > max_altitude;
horizontal_violation = norm(x_est(1:2)) > max_horizontal;

violation = altitude_violation || horizontal_violation;

end

function valid = is_valid_mode_transition(from_mode, to_mode)
% Convert to lower case & trim to avoid false mismatches
from_mode = lower(strtrim(from_mode));
to_mode   = lower(strtrim(to_mode));
% Validate mode transitions based on state machine rules

% Define valid transitions (following tutorial methodology)
valid_transitions = {
    'disarmed', {'stabilized'};
    'stabilized', {'altitude', 'return_to_home', 'disarmed', 'emergency_land'};
    'altitude', {'position', 'stabilized', 'return_to_home', 'emergency_land'};
    'position', {'trajectory', 'altitude', 'return_to_home', 'emergency_land'};
    'trajectory', {'position', 'return_to_home', 'emergency_land'};
    'return_to_home', {'stabilized', 'emergency_land'};
    'emergency_land', {'disarmed'};
};

% Check if transition is valid
valid = false;
for i = 1:size(valid_transitions, 1)
    if strcmp(from_mode, valid_transitions{i, 1})
        if any(strcmp(to_mode, valid_transitions{i, 2}))
            valid = true;
            break;
        end
    end
end

end

function flight_mgmt = initialize_mode_specific_parameters(flight_mgmt, x_est)
% Initialize parameters specific to the new flight mode
% x_est: current state estimate [pos(3); vel(3); att(3)]

% Extract current position and altitude from state estimate
current_position = x_est(1:3);          % [x, y, z] in NED frame
current_altitude = -x_est(3);           % Convert NED down to altitude up

switch flight_mgmt.current_mode
    case 'stabilized'
        % Reset all integrators for manual control
        flight_mgmt.control_integrators_enabled = false;
        
    case 'altitude'
        % Enable altitude control integrator
        flight_mgmt.altitude_hold_enabled = true;
        flight_mgmt.altitude_reference = current_altitude;
        
    case 'position'
        % Enable position hold
        flight_mgmt.position_hold_enabled = true;
        flight_mgmt.position_reference = current_position;
        
    case 'trajectory'
        % Enable autonomous trajectory following
        flight_mgmt.trajectory_mode_enabled = true;
        flight_mgmt.waypoint_tolerance = 2.0; % meters
        
    case 'return_to_home'
        % Set home position as target
        flight_mgmt.home_position = [0; 0; 0]; % Assume origin is home
        flight_mgmt.return_altitude = 20; % Return at 20m altitude
        
    case 'emergency_land'
        % Emergency landing parameters
        flight_mgmt.emergency_descent_rate = -1.0; % m/s
        flight_mgmt.emergency_landing_enabled = true;
end

end

function status = assess_overall_system_status(flight_mgmt, x_est)
% Assess overall system health and status

% Start with nominal status
status = 'NOMINAL';

% Check for degraded conditions
if ~flight_mgmt.sensor_health.gps && ...
   (strcmp(flight_mgmt.current_mode, 'position') || strcmp(flight_mgmt.current_mode, 'trajectory'))
    status = 'DEGRADED - GPS UNAVAILABLE';
end

if ~flight_mgmt.sensor_health.baro && ...
   ~strcmp(flight_mgmt.current_mode, 'stabilized')
    status = 'DEGRADED - ALTITUDE SENSOR UNAVAILABLE';
end

% Check for emergency conditions
if strcmp(flight_mgmt.current_mode, 'return_to_home')
    status = 'EMERGENCY - RETURNING HOME';
elseif strcmp(flight_mgmt.current_mode, 'emergency_land')
    status = 'CRITICAL - EMERGENCY LANDING';
end

% Check for extreme attitudes or velocities
if norm(x_est(7:8)) > deg2rad(45) % > 45° roll/pitch
    status = 'WARNING - EXTREME ATTITUDE';
end

if norm(x_est(4:6)) > 20 % > 20 m/s velocity
    status = 'WARNING - HIGH VELOCITY';
end

end

function quality_ok = check_gps_quality(flight_mgmt)
% Check GPS measurement quality

% Default to good quality
quality_ok = true;

% Check GPS-specific quality metrics
if isfield(flight_mgmt, 'gps_hdop')
    quality_ok = quality_ok && (flight_mgmt.gps_hdop < 3.0); % HDOP threshold
end

if isfield(flight_mgmt, 'gps_satellites')
    quality_ok = quality_ok && (flight_mgmt.gps_satellites >= 6); % Minimum satellites
end

end

function quality_ok = check_baro_quality(flight_mgmt)
% Check barometer measurement quality

% Default to good quality
quality_ok = true;

% Check for reasonable pressure readings
if isfield(flight_mgmt, 'baro_pressure')
    % Pressure should be reasonable (800-1100 hPa)
    pressure_hpa = flight_mgmt.baro_pressure / 100; % Convert Pa to hPa
    quality_ok = quality_ok && (pressure_hpa > 800) && (pressure_hpa < 1100);
end

end

function quality_ok = check_mag_quality(flight_mgmt)
% Check magnetometer measurement quality

% Default to good quality
quality_ok = true;

% Check magnetic field strength
if isfield(flight_mgmt, 'mag_vector')
    mag_strength = norm(flight_mgmt.mag_vector);
    % Earth's magnetic field is typically 25-65 µT
    quality_ok = quality_ok && (mag_strength > 20) && (mag_strength < 80);
end

end