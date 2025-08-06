function logger = enhanced_ekf_logger(log_filename)
% enhanced_ekf_logger - Comprehensive logging system for enhanced EKF simulation
% Creates detailed logs for performance analysis and parameter tuning
%
% Usage:
%   logger = enhanced_ekf_logger('simulation_log.txt');
%   logger.log('INFO', 'Simulation started');
%   logger.log_performance(t, x_true, x_est, error_metrics);
%   logger.close();

if nargin < 1
    log_filename = sprintf('enhanced_ekf_log_%s.txt', datestr(now, 'yyyy-mm-dd_HH-MM-SS'));
end

% Initialize log file
fid = fopen(log_filename, 'w');
if fid == -1
    error('Could not create log file: %s', log_filename);
end

% Write header
fprintf(fid, '=================================================================\n');
fprintf(fid, 'ENHANCED EKF QUADCOPTER AUTOPILOT - DETAILED SIMULATION LOG\n');
fprintf(fid, 'Based on MATLAB Simulink Autonomous Flight Design Tutorial\n');
fprintf(fid, '=================================================================\n');
fprintf(fid, 'Log Started: %s\n', datestr(now));
fprintf(fid, 'MATLAB Version: %s\n', version);
fprintf(fid, '=================================================================\n\n');

% Initialize performance tracking
perf_data = struct();
perf_data.position_errors = [];
perf_data.velocity_errors = [];
perf_data.attitude_errors = [];
perf_data.sensor_updates = [];
perf_data.mode_transitions = {};
perf_data.emergency_events = {};
perf_data.sensor_failures = {};

% Create logger object
logger = struct();
logger.fid = fid;
logger.filename = log_filename;
logger.perf_data = perf_data;
logger.start_time = now;

% Logger methods
logger.log = @(level, message) log_message(fid, level, message);
logger.log_system_status = @(status) log_system_status(fid, status);
logger.log_flight_mode = @(mode, time, reason) log_flight_mode(fid, perf_data, mode, time, reason);
logger.log_sensor_update = @(sensor_type, time, data, health) log_sensor_update(fid, perf_data, sensor_type, time, data, health);
logger.log_performance = @(time, x_true, x_est, control_input) log_performance_data(fid, perf_data, time, x_true, x_est, control_input);
logger.log_emergency = @(event_type, time, details) log_emergency_event(fid, perf_data, event_type, time, details);
logger.log_ekf_metrics = @(time, innovation, covariance, adaptive_noise) log_ekf_metrics(fid, time, innovation, covariance, adaptive_noise);
logger.generate_summary = @() generate_simulation_summary(fid, perf_data, log_filename);
logger.close = @() close_logger(fid, perf_data, log_filename);

fprintf('Enhanced EKF Logger initialized: %s\n', log_filename);

end

%% LOGGING FUNCTIONS

function log_message(fid, level, message)
timestamp = datestr(now, 'HH:MM:SS.FFF');
fprintf(fid, '[%s] %s: %s\n', timestamp, level, message);
if strcmp(level, 'ERROR') || strcmp(level, 'WARNING')
    fprintf('[%s] %s: %s\n', timestamp, level, message); % Also display to console
end
end

function log_system_status(fid, status)
fprintf(fid, '\n--- SYSTEM STATUS ---\n');
fields = fieldnames(status);
for i = 1:length(fields)
    field_value = status.(fields{i});
    if isnumeric(field_value)
        value_str = mat2str(field_value);
    elseif islogical(field_value)
        value_str = mat2str(field_value);
    elseif ischar(field_value) || isstring(field_value)
        value_str = char(field_value);
    elseif isstruct(field_value)
        % Handle struct fields
        struct_fields = fieldnames(field_value);
        value_str = '{';
        for j = 1:length(struct_fields)
            if j > 1, value_str = [value_str, ', ']; end
            sub_value = field_value.(struct_fields{j});
            if isnumeric(sub_value)
                value_str = [value_str, struct_fields{j}, ':', mat2str(sub_value)];
            elseif islogical(sub_value)
                value_str = [value_str, struct_fields{j}, ':', mat2str(sub_value)];
            elseif ischar(sub_value) || isstring(sub_value)
                value_str = [value_str, struct_fields{j}, ':', char(sub_value)];
            else
                value_str = [value_str, struct_fields{j}, ':', class(sub_value)];
            end
        end
        value_str = [value_str, '}'];
    elseif iscell(field_value)
        value_str = '{cell array}';
    else
        value_str = class(field_value);
    end
    fprintf(fid, '%s: %s\n', fields{i}, value_str);
end
fprintf(fid, '--------------------\n\n');
end

function log_flight_mode(fid, perf_data, mode, time, reason)
fprintf(fid, '[%.3f] FLIGHT MODE: %s (%s)\n', time, upper(mode), reason);
perf_data.mode_transitions{end+1} = struct('time', time, 'mode', mode, 'reason', reason);
end

function log_sensor_update(fid, perf_data, sensor_type, time, data, health)
if health
    status = 'OK';
else
    status = 'FAIL';
end
fprintf(fid, '[%.3f] SENSOR-%s: %s [%s]\n', time, upper(sensor_type), mat2str(data, 3), status);
perf_data.sensor_updates = [perf_data.sensor_updates; time, health];
end

function log_performance_data(fid, perf_data, time, x_true, x_est, control_input)
% Calculate errors
pos_error = norm(x_true(1:3) - x_est(1:3));
vel_error = norm(x_true(4:6) - x_est(4:6));
att_error = norm(x_true(7:9) - x_est(7:9));

% Store for analysis
perf_data.position_errors = [perf_data.position_errors; time, pos_error];
perf_data.velocity_errors = [perf_data.velocity_errors; time, vel_error];
perf_data.attitude_errors = [perf_data.attitude_errors; time, att_error];

% Log every 5 seconds for readability
if mod(time, 5.0) < 0.01
    fprintf(fid, '[%.1f] PERFORMANCE: Pos=%.3fm, Vel=%.3fm/s, Att=%.3f°, Thrust=%.2fN\n', ...
        time, pos_error, vel_error, rad2deg(att_error), control_input(1));
end
end

function log_emergency_event(fid, perf_data, event_type, time, details)
fprintf(fid, '[%.3f] EMERGENCY: %s - %s\n', time, upper(event_type), details);
perf_data.emergency_events{end+1} = struct('time', time, 'type', event_type, 'details', details);
end

function log_ekf_metrics(fid, time, innovation, covariance, adaptive_noise)
fprintf(fid, '[%.3f] EKF: Innovation=%.3f, Trace(P)=%.3f, AdaptiveNoise=%.3f\n', ...
    time, norm(innovation), trace(covariance), adaptive_noise);
end

function generate_simulation_summary(fid, perf_data, filename)
fprintf(fid, '\n\n=================================================================\n');
fprintf(fid, 'SIMULATION SUMMARY AND ANALYSIS\n');
fprintf(fid, '=================================================================\n');

% Performance statistics
if ~isempty(perf_data.position_errors)
    pos_errors = perf_data.position_errors(:,2);
    fprintf(fid, 'POSITION ACCURACY:\n');
    fprintf(fid, '  Mean Error: %.3f m\n', mean(pos_errors));
    fprintf(fid, '  RMS Error:  %.3f m\n', sqrt(mean(pos_errors.^2)));
    fprintf(fid, '  Max Error:  %.3f m\n', max(pos_errors));
    fprintf(fid, '  Min Error:  %.3f m\n', min(pos_errors));
    fprintf(fid, '  Std Dev:    %.3f m\n', std(pos_errors));
end

if ~isempty(perf_data.velocity_errors)
    vel_errors = perf_data.velocity_errors(:,2);
    fprintf(fid, '\nVELOCITY ACCURACY:\n');
    fprintf(fid, '  Mean Error: %.3f m/s\n', mean(vel_errors));
    fprintf(fid, '  RMS Error:  %.3f m/s\n', sqrt(mean(vel_errors.^2)));
    fprintf(fid, '  Max Error:  %.3f m/s\n', max(vel_errors));
end

if ~isempty(perf_data.attitude_errors)
    att_errors = perf_data.attitude_errors(:,2);
    fprintf(fid, '\nATTITUDE ACCURACY:\n');
    fprintf(fid, '  Mean Error: %.3f°\n', rad2deg(mean(att_errors)));
    fprintf(fid, '  RMS Error:  %.3f°\n', rad2deg(sqrt(mean(att_errors.^2))));
    fprintf(fid, '  Max Error:  %.3f°\n', rad2deg(max(att_errors)));
end

% Mode transitions analysis
fprintf(fid, '\nFLIGHT MODE TRANSITIONS:\n');
for i = 1:length(perf_data.mode_transitions)
    trans = perf_data.mode_transitions{i};
    fprintf(fid, '  t=%.1fs: %s (%s)\n', trans.time, trans.mode, trans.reason);
end

% Emergency events analysis
fprintf(fid, '\nEMERGENCY EVENTS:\n');
if isempty(perf_data.emergency_events)
    fprintf(fid, '  None recorded.\n');
else
    for i = 1:length(perf_data.emergency_events)
        event = perf_data.emergency_events{i};
        fprintf(fid, '  t=%.1fs: %s - %s\n', event.time, event.type, event.details);
    end
end

% Sensor performance
if ~isempty(perf_data.sensor_updates)
    sensor_health = perf_data.sensor_updates(:,2);
    fprintf(fid, '\nSENSOR PERFORMANCE:\n');
    fprintf(fid, '  Total Updates: %d\n', length(sensor_health));
    fprintf(fid, '  Success Rate:  %.1f%%\n', mean(sensor_health) * 100);
    fprintf(fid, '  Failed Updates: %d\n', sum(~sensor_health));
end

fprintf(fid, '\n=================================================================\n');
fprintf(fid, 'Log file saved: %s\n', filename);
fprintf(fid, 'Analysis complete: %s\n', datestr(now));
fprintf(fid, '=================================================================\n');
end

function close_logger(fid, perf_data, filename)
generate_simulation_summary(fid, perf_data, filename);
fclose(fid);
fprintf('Enhanced EKF simulation log saved: %s\n', filename);
fprintf('Performance summary generated.\n');
end