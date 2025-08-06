function [wp_idx, waypoint_switched] = update_waypoint_management(x_est, waypoints, wp_idx, current_mode, current_time)
% update_waypoint_management - Enhanced waypoint management system
% Based on MATLAB Simulink Autonomous Flight Design Tutorial methodology
%
% Implements intelligent waypoint switching logic that adapts to:
% - Current flight mode capabilities
% - Vehicle dynamics and performance
% - Mission requirements and safety
% - Path geometry and turn anticipation
%
% Features:
% - Mode-dependent switching criteria
% - Predictive switching for smooth trajectories
% - Safety checks and abort conditions
% - Performance optimization
%
% Inputs:
%   x_est - Current state estimate [pos(3); vel(3); att(3)]
%   waypoints - Mission waypoints [3 x N]
%   wp_idx - Current waypoint index
%   current_mode - Current flight mode
%   current_time - Current simulation time
%
% Outputs:
%   wp_idx - Updated waypoint index
%   waypoint_switched - Boolean indicating if waypoint was switched

waypoint_switched = false;

% Safety check: ensure valid waypoint index
if wp_idx > size(waypoints, 2)
    return; % Mission complete
end

% Extract current state
pos = x_est(1:3);
vel = x_est(4:6);
att = x_est(7:9);

% Current target waypoint
current_wp = waypoints(:, wp_idx);

% Calculate distance to current waypoint
distance_to_wp = norm(pos - current_wp);

%% MODE-DEPENDENT WAYPOINT SWITCHING LOGIC
% Different flight modes have different switching criteria

switch lower(current_mode)
    case 'stabilized'
        % Manual mode - simple distance-based switching
        [wp_idx, waypoint_switched] = stabilized_mode_switching(...
            wp_idx, pos, current_wp, waypoints, distance_to_wp);
        
    case 'altitude'
        % Altitude mode - consider vertical performance
        [wp_idx, waypoint_switched] = altitude_mode_switching(...
            wp_idx, pos, vel, current_wp, waypoints, distance_to_wp);
        
    case 'position'
        % Position mode - 3D position accuracy
        [wp_idx, waypoint_switched] = position_mode_switching(...
            wp_idx, pos, vel, current_wp, waypoints, distance_to_wp);
        
    case 'trajectory'
        % Trajectory mode - predictive switching
        [wp_idx, waypoint_switched] = trajectory_mode_switching(...
            wp_idx, pos, vel, current_wp, waypoints, distance_to_wp, current_time);
        
    case 'return_to_home'
        % Return to home - single target
        home_pos = [0; 0; -20]; % Home at origin, 20m altitude
        if norm(pos - home_pos) < 3.0
            waypoint_switched = true;
            fprintf('t=%.1fs: Arrived at home position\n', current_time);
        end
        
    case 'emergency_land'
        % Emergency landing - descend at current position
        if -pos(3) < 1.0 && abs(vel(3)) < 0.5 % Close to ground with low velocity
            waypoint_switched = true;
            fprintf('t=%.1fs: Emergency landing complete\n', current_time);
        end
        
    otherwise
        % Default switching logic
        [wp_idx, waypoint_switched] = default_switching(...
            wp_idx, pos, current_wp, waypoints, distance_to_wp);
end

%% SAFETY AND VALIDATION CHECKS
if waypoint_switched
    % Validate waypoint switch
    if wp_idx <= size(waypoints, 2)
        new_wp = waypoints(:, wp_idx);
        
        % Check if new waypoint is reasonable
        distance_to_new_wp = norm(pos - new_wp);
        if distance_to_new_wp > 500 % More than 500m jump
            fprintf('Warning: Large waypoint jump detected (%.1fm). Validating...\n', distance_to_new_wp);
            if ~validate_waypoint_switch(pos, current_wp, new_wp)
                wp_idx = wp_idx - 1; % Revert switch
                waypoint_switched = false;
                fprintf('Warning: Waypoint switch rejected for safety.\n');
            end
        end
        
        % Log successful waypoint switch
        if waypoint_switched
            fprintf('t=%.1fs: Waypoint %d → %d (%s mode) - Distance: %.1fm\n', ...
                current_time, wp_idx-1, wp_idx, current_mode, distance_to_new_wp);
        end
    end
end

end

%% MODE-SPECIFIC SWITCHING FUNCTIONS

function [wp_idx, switched] = stabilized_mode_switching(wp_idx, pos, current_wp, waypoints, distance_to_wp)
% Stabilized mode: Simple distance-based switching for manual flight

switched = false;
switching_distance = 8.0; % Larger distance for manual control

if distance_to_wp < switching_distance && wp_idx < size(waypoints, 2)
    wp_idx = wp_idx + 1;
    switched = true;
end

end

function [wp_idx, switched] = altitude_mode_switching(wp_idx, pos, vel, current_wp, waypoints, distance_to_wp)
% Altitude mode: Consider vertical performance and altitude changes

switched = false;
base_switching_distance = 5.0;

% Adjust switching distance based on altitude change
if wp_idx < size(waypoints, 2)
    next_wp = waypoints(:, wp_idx + 1);
    altitude_change = abs(next_wp(3) - current_wp(3));
    
    % Increase switching distance for large altitude changes
    switching_distance = base_switching_distance + 0.5 * altitude_change;
else
    switching_distance = base_switching_distance;
end

% Consider vertical velocity for smooth altitude transitions
vertical_velocity = abs(vel(3));
if vertical_velocity > 2.0 % High vertical velocity
    switching_distance = switching_distance * 1.5; % Switch earlier
end

if distance_to_wp < switching_distance && wp_idx < size(waypoints, 2)
    wp_idx = wp_idx + 1;
    switched = true;
end

end

function [wp_idx, switched] = position_mode_switching(wp_idx, pos, vel, current_wp, waypoints, distance_to_wp)
% Position mode: 3D position accuracy with velocity consideration

switched = false;
base_switching_distance = 3.0; % Tighter tolerance for position mode

% Adjust based on vehicle velocity
velocity_magnitude = norm(vel);
if velocity_magnitude > 5.0
    % High speed - switch earlier to allow deceleration
    switching_distance = base_switching_distance + velocity_magnitude * 0.5;
else
    switching_distance = base_switching_distance;
end

% Use progress-based switching for better path following
if wp_idx > 1 && wp_idx < size(waypoints, 2)
    switched = progress_based_switching(wp_idx, pos, waypoints, switching_distance);
    if switched
        wp_idx = wp_idx + 1;
    end
elseif distance_to_wp < switching_distance && wp_idx < size(waypoints, 2)
    wp_idx = wp_idx + 1;
    switched = true;
end

end

function [wp_idx, switched] = trajectory_mode_switching(wp_idx, pos, vel, current_wp, waypoints, distance_to_wp, current_time)
% Trajectory mode: Predictive switching with look-ahead

switched = false;
base_switching_distance = 2.0; % Tight tolerance for trajectory following

% Predictive switching based on trajectory geometry
if wp_idx < size(waypoints, 2)
    next_wp = waypoints(:, wp_idx + 1);
    
    % Analyze turn angle for predictive switching
    if wp_idx > 1
        prev_wp = waypoints(:, wp_idx - 1);
        turn_angle = calculate_turn_angle(prev_wp, current_wp, next_wp);
        
        % Adjust switching distance based on turn sharpness
        if abs(turn_angle) > pi/3 % Sharp turn (>60°)
            switching_distance = base_switching_distance + 3.0; % Switch early
        elseif abs(turn_angle) > pi/6 % Moderate turn (>30°)
            switching_distance = base_switching_distance + 1.5;
        else
            switching_distance = base_switching_distance;
        end
    else
        switching_distance = base_switching_distance;
    end
    
    % Consider velocity for dynamic switching
    velocity_magnitude = norm(vel);
    velocity_adjustment = min(velocity_magnitude * 0.3, 2.0); % Max 2m adjustment
    switching_distance = switching_distance + velocity_adjustment;
    
    % Use advanced progress-based switching
    switched = advanced_progress_switching(wp_idx, pos, vel, waypoints, switching_distance);
    if switched
        wp_idx = wp_idx + 1;
    end
else
    % Final waypoint - use simple distance
    if distance_to_wp < base_switching_distance
        wp_idx = wp_idx + 1;
        switched = true;
    end
end

end

function [wp_idx, switched] = default_switching(wp_idx, pos, current_wp, waypoints, distance_to_wp)
% Default switching logic for unknown modes

switched = false;
switching_distance = 5.0; % Conservative distance

if distance_to_wp < switching_distance && wp_idx < size(waypoints, 2)
    wp_idx = wp_idx + 1;
    switched = true;
end

end

%% ADVANCED SWITCHING ALGORITHMS

function switched = progress_based_switching(wp_idx, pos, waypoints, switching_distance)
% Progress-based switching for smoother path following

switched = false;

if wp_idx > 1 && wp_idx < size(waypoints, 2)
    prev_wp = waypoints(:, wp_idx - 1);
    current_wp = waypoints(:, wp_idx);
    
    % Vector from previous to current waypoint
    segment_vector = current_wp - prev_wp;
    segment_length = norm(segment_vector);
    
    if segment_length > 0.1 % Avoid division by zero
        % Vector from previous waypoint to current position
        position_vector = pos - prev_wp;
        
        % Project position onto path segment
        progress = dot(segment_vector, position_vector) / (segment_length^2);
        progress = max(0, min(progress, 1)); % Clamp to [0, 1]
        
        % Distance along path
        distance_along_path = progress * segment_length;
        
        % Switch when close to end of segment
        remaining_distance = segment_length - distance_along_path;
        if remaining_distance < switching_distance
            switched = true;
        end
    end
end

end

function switched = advanced_progress_switching(wp_idx, pos, vel, waypoints, switching_distance)
% Advanced progress-based switching with velocity prediction - MUCH MORE CONSERVATIVE

switched = false;

if wp_idx > 1 && wp_idx < size(waypoints, 2)
    prev_wp = waypoints(:, wp_idx - 1);
    current_wp = waypoints(:, wp_idx);
    next_wp = waypoints(:, wp_idx + 1);
    
    % Current segment analysis
    segment_vector = current_wp - prev_wp;
    segment_length = norm(segment_vector);
    
    if segment_length > 0.1
        % Current progress calculation
        position_vector = pos - prev_wp;
        progress = dot(segment_vector, position_vector) / (segment_length^2);
        progress = max(0, min(progress, 1));
        
        % Velocity-based prediction - more conservative
        velocity_magnitude = norm(vel);
        
        % Check if velocity is too high - don't switch if moving too fast
        if velocity_magnitude > 15.0
            % Too fast - don't switch yet, let the drone slow down
            switched = false;
            return;
        end
        
        if velocity_magnitude > 0.5
            % Predict time to reach waypoint
            remaining_distance = segment_length * (1 - progress);
            time_to_waypoint = remaining_distance / velocity_magnitude;
            
            % Look ahead for turn preparation - more conservative
            if wp_idx < size(waypoints, 2)
                turn_angle = calculate_turn_angle(prev_wp, current_wp, next_wp);
                
                % Much more conservative turn handling
                if abs(turn_angle) > pi/6 % Significant turn
                    % More conservative deceleration assumption (1.5 m/s² instead of 2.0)
                    required_decel_distance = velocity_magnitude^2 / (2 * 1.5);
                    
                    % Only switch if we're very close to the required deceleration point
                    if remaining_distance < max(switching_distance * 1.5, required_decel_distance)
                        % Check velocity again - don't switch if still moving too fast into a turn
                        if velocity_magnitude < 10.0 || abs(turn_angle) < pi/4
                            switched = true;
                        end
                    end
                else
                    % For gentle turns, use standard switching logic but with higher threshold
                    if remaining_distance < switching_distance * 1.2
                        switched = true;
                    end
                end
            else
                % Standard progress-based switching with increased threshold
                if remaining_distance < switching_distance * 1.2
                    switched = true;
                end
            end
        else
            % Low velocity - use simple distance with slightly increased threshold
            distance_to_wp = norm(pos - current_wp);
            if distance_to_wp < switching_distance * 1.1
                switched = true;
            end
        end
        
        % Additional cross-track error check - don't switch if we're too far off path
        cross_track_vector = position_vector - progress * segment_vector;
        cross_track_error = norm(cross_track_vector);
        
        if cross_track_error > 8.0 && progress < 0.9
            % Too far off path - don't switch yet
            switched = false;
        end
    end
end

end

%% UTILITY FUNCTIONS

function turn_angle = calculate_turn_angle(wp1, wp2, wp3)
% Calculate the turn angle at wp2 when going from wp1 to wp3

% Vectors of the path segments
v1 = wp2 - wp1; % Vector to current waypoint
v2 = wp3 - wp2; % Vector from current to next waypoint

% Normalize vectors
v1_norm = v1 / norm(v1);
v2_norm = v2 / norm(v2);

% Calculate angle between vectors
cos_angle = dot(v1_norm, v2_norm);
cos_angle = max(-1, min(1, cos_angle)); % Clamp to avoid numerical issues

turn_angle = acos(cos_angle);

% Determine turn direction (optional)
cross_product = cross([v1_norm; 0], [v2_norm; 0]);
if cross_product(3) < 0
    turn_angle = -turn_angle; % Right turn
end

end

function valid = validate_waypoint_switch(current_pos, old_wp, new_wp)
% Validate if waypoint switch is safe and reasonable

valid = true;

% Check 1: Maximum distance jump
max_distance_jump = 200; % meters
distance_jump = norm(new_wp - old_wp);
if distance_jump > max_distance_jump
    valid = false;
    return;
end

% Check 2: Maximum altitude change
max_altitude_change = 50; % meters
altitude_change = abs(new_wp(3) - old_wp(3));
if altitude_change > max_altitude_change
    valid = false;
    return;
end

% Check 3: Reasonable progression (not going backwards significantly)
current_to_old = norm(current_pos - old_wp);
current_to_new = norm(current_pos - new_wp);
if current_to_new > 2 * current_to_old % New waypoint is much farther
    % Additional check: make sure it's forward progress
    direction_to_old = old_wp - current_pos;
    direction_to_new = new_wp - current_pos;
    
    if norm(direction_to_old) > 0 && norm(direction_to_new) > 0
        cos_angle = dot(direction_to_old, direction_to_new) / ...
                   (norm(direction_to_old) * norm(direction_to_new));
        
        if cos_angle < -0.5 % More than 120° backwards
            valid = false;
            return;
        end
    end
end

% Check 4: Geofence constraints
max_horizontal_distance = 300; % meters from origin
max_altitude = 100; % meters
if norm(new_wp(1:2)) > max_horizontal_distance || -new_wp(3) > max_altitude
    valid = false;
    return;
end

end