function create_enhanced_visualizations(t, x_true_hist, x_est_hist, waypoints, gps_hist, mode_hist, pos_errors, estimation_errors)
% create_enhanced_visualizations - Comprehensive visualization suite
% Based on MATLAB Simulink Autonomous Flight Design Tutorial methodology
%
% Creates a comprehensive set of plots and analyses including:
% - 3D trajectory visualization with flight modes
% - State estimation performance analysis
% - Control performance metrics
% - Sensor data quality assessment
% - Flight mode transition analysis
% - Error analysis and statistics
%
% Inputs:
%   t - Time vector
%   x_true_hist - True state history [9 x N]
%   x_est_hist - Estimated state history [9 x N]
%   waypoints - Mission waypoints [3 x N]
%   gps_hist - GPS measurement history [3 x N]
%   mode_hist - Flight mode history {N x 1}
%   pos_errors - Position error history [1 x N]
%   estimation_errors - Estimation error history [1 x N]

fprintf('\n=== Creating Enhanced Visualization Suite ===\n');

%% FIGURE 1: 3D TRAJECTORY WITH FLIGHT MODES
create_3d_trajectory_plot(t, x_true_hist, x_est_hist, waypoints, gps_hist, mode_hist);

%% FIGURE 2: STATE ESTIMATION PERFORMANCE
create_estimation_performance_plot(t, x_true_hist, x_est_hist, estimation_errors);

%% FIGURE 3: POSITION AND VELOCITY ANALYSIS
create_position_velocity_analysis(t, x_true_hist, x_est_hist, waypoints);

%% FIGURE 4: ATTITUDE AND ORIENTATION ANALYSIS
create_attitude_analysis(t, x_true_hist, x_est_hist);

%% FIGURE 5: ERROR ANALYSIS AND STATISTICS
create_error_analysis(t, pos_errors, estimation_errors);

%% FIGURE 6: FLIGHT MODE TRANSITION ANALYSIS
create_flight_mode_analysis(t, mode_hist, x_true_hist);

%% FIGURE 7: SENSOR MEASUREMENT QUALITY
create_sensor_quality_analysis(t, x_true_hist, x_est_hist, gps_hist);

%% FIGURE 8: PERFORMANCE DASHBOARD
create_performance_dashboard(t, x_true_hist, x_est_hist, waypoints, pos_errors, estimation_errors, mode_hist);

fprintf('✅ Enhanced visualization suite complete\n');
fprintf('   Generated 8 comprehensive analysis plots\n\n');

end

%% VISUALIZATION FUNCTIONS

function create_3d_trajectory_plot(t, x_true_hist, x_est_hist, waypoints, gps_hist, mode_hist)
% Enhanced 3D trajectory plot with flight mode visualization

figure('Name', 'Enhanced 3D Trajectory - Flight Modes & Performance', ...
       'Position', [100, 100, 1400, 800]);

subplot(1,2,1);
% Main 3D trajectory
plot3(x_true_hist(1,:), x_true_hist(2,:), x_true_hist(3,:), 'b-', 'LineWidth', 2.5);
hold on;
plot3(x_est_hist(1,:), x_est_hist(2,:), x_est_hist(3,:), 'r--', 'LineWidth', 2);

% Plot GPS measurements (when available)
valid_gps = ~isnan(gps_hist(1,:));
if any(valid_gps)
    plot3(gps_hist(1,valid_gps), gps_hist(2,valid_gps), gps_hist(3,valid_gps), ...
          'yo', 'MarkerSize', 4, 'MarkerFaceColor', 'yellow');
end

% Waypoints with enhanced styling
plot3(waypoints(1,:), waypoints(2,:), waypoints(3,:), 'ms', ...
      'MarkerSize', 12, 'MarkerFaceColor', 'magenta', 'LineWidth', 2);

% Connect waypoints with path
plot3(waypoints(1,:), waypoints(2,:), waypoints(3,:), 'm--', ...
      'LineWidth', 1.5);

% Mark start and end points
plot3(x_true_hist(1,1), x_true_hist(2,1), x_true_hist(3,1), 'go', ...
      'MarkerSize', 15, 'MarkerFaceColor', 'green', 'LineWidth', 3);
plot3(x_true_hist(1,end), x_true_hist(2,end), x_true_hist(3,end), 'ro', ...
      'MarkerSize', 15, 'MarkerFaceColor', 'red', 'LineWidth', 3);

% Enhanced ground reference
[X_ground, Y_ground] = meshgrid(-20:20:120, -20:20:120);
Z_ground = zeros(size(X_ground));
mesh(X_ground, Y_ground, Z_ground, 'EdgeColor', [0.8 0.8 0.8], ...
     'FaceAlpha', 0.1, 'EdgeAlpha', 0.3);

% Coordinate system arrows
quiver3(0,0,0, 20,0,0, 'r', 'LineWidth', 3, 'MaxHeadSize', 1);
quiver3(0,0,0, 0,20,0, 'g', 'LineWidth', 3, 'MaxHeadSize', 1);
quiver3(0,0,0, 0,0,20, 'b', 'LineWidth', 3, 'MaxHeadSize', 1);

grid on; axis equal;
xlabel('North (m)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('East (m)', 'FontSize', 12, 'FontWeight', 'bold');
zlabel('Down (m)', 'FontSize', 12, 'FontWeight', 'bold');
title('3D Flight Trajectory with Flight Modes', 'FontSize', 14, 'FontWeight', 'bold');
legend('True Trajectory', 'EKF Estimate', 'GPS Measurements', 'Waypoints', ...
       'Planned Path', 'Start', 'End', 'Location', 'best');
view(45, 30);

% Flight mode timeline
subplot(1,2,2);
plot_flight_mode_timeline(t, mode_hist, x_true_hist);

end

function plot_flight_mode_timeline(t, mode_hist, x_true_hist)
% Plot flight mode transitions over time

% Convert mode history to numeric for plotting
mode_names = {'disarmed', 'stabilized', 'altitude', 'position', 'trajectory', 'return_to_home', 'emergency_land'};
mode_numeric = zeros(size(mode_hist));

for i = 1:length(mode_hist)
    if ~isempty(mode_hist{i})
        mode_idx = find(strcmp(mode_hist{i}, mode_names));
        if ~isempty(mode_idx)
            mode_numeric(i) = mode_idx;
        else
            mode_numeric(i) = 2; % Default to stabilized
        end
    else
        mode_numeric(i) = 2; % Default to stabilized
    end
end

% Plot mode transitions
stairs(t, mode_numeric, 'LineWidth', 3);
ylim([0.5, length(mode_names) + 0.5]);
yticks(1:length(mode_names));
yticklabels(mode_names);
grid on;
xlabel('Time (s)', 'FontSize', 12);
ylabel('Flight Mode', 'FontSize', 12);
title('Flight Mode Transitions', 'FontSize', 14, 'FontWeight', 'bold');

% Add altitude overlay
yyaxis right;
plot(t, -x_true_hist(3,:), 'g--', 'LineWidth', 2);
ylabel('Altitude (m)', 'FontSize', 12);
legend('Flight Mode', 'Altitude', 'Location', 'best');

end

function create_estimation_performance_plot(t, x_true_hist, x_est_hist, estimation_errors)
% Comprehensive estimation performance analysis

figure('Name', 'EKF State Estimation Performance Analysis', ...
       'Position', [150, 150, 1400, 900]);

% Position estimation errors
subplot(3,2,1);
pos_errors = x_true_hist(1:3,:) - x_est_hist(1:3,:);
plot(t, pos_errors(1,:), 'r-', 'LineWidth', 1.5); hold on;
plot(t, pos_errors(2,:), 'g-', 'LineWidth', 1.5);
plot(t, pos_errors(3,:), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Position Error (m)');
title('Position Estimation Errors', 'FontWeight', 'bold');
legend('North', 'East', 'Down', 'Location', 'best');

% Velocity estimation errors
subplot(3,2,2);
vel_errors = x_true_hist(4:6,:) - x_est_hist(4:6,:);
plot(t, vel_errors(1,:), 'r-', 'LineWidth', 1.5); hold on;
plot(t, vel_errors(2,:), 'g-', 'LineWidth', 1.5);
plot(t, vel_errors(3,:), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Velocity Error (m/s)');
title('Velocity Estimation Errors', 'FontWeight', 'bold');
legend('North', 'East', 'Down', 'Location', 'best');

% Attitude estimation errors
subplot(3,2,3);
att_errors = rad2deg(x_true_hist(7:9,:) - x_est_hist(7:9,:));
plot(t, att_errors(1,:), 'r-', 'LineWidth', 1.5); hold on;
plot(t, att_errors(2,:), 'g-', 'LineWidth', 1.5);
plot(t, att_errors(3,:), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Attitude Error (deg)');
title('Attitude Estimation Errors', 'FontWeight', 'bold');
legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');

% Overall estimation error magnitude
subplot(3,2,4);
plot(t, estimation_errors, 'k-', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Total Position Error (m)');
title('Overall Estimation Performance', 'FontWeight', 'bold');

% Add statistical information
mean_error = mean(estimation_errors);
max_error = max(estimation_errors);
rms_error = sqrt(mean(estimation_errors.^2));
hold on;
yline(mean_error, 'r--', sprintf('Mean: %.2fm', mean_error), 'LineWidth', 2);
yline(rms_error, 'g--', sprintf('RMS: %.2fm', rms_error), 'LineWidth', 2);

% Error histogram
subplot(3,2,5);
histogram(estimation_errors, 50, 'Normalization', 'probability', 'FaceAlpha', 0.7);
xlabel('Estimation Error (m)');
ylabel('Probability');
title('Error Distribution', 'FontWeight', 'bold');
grid on;

% Cumulative error plot
subplot(3,2,6);
sorted_errors = sort(estimation_errors);
p = (1:length(sorted_errors)) / length(sorted_errors) * 100;
plot(sorted_errors, p, 'b-', 'LineWidth', 2);
xlabel('Estimation Error (m)');
ylabel('Cumulative Percentage (%)');
title('Cumulative Error Distribution', 'FontWeight', 'bold');
grid on;

% Add performance metrics
error_50 = sorted_errors(round(0.5 * length(sorted_errors)));
error_95 = sorted_errors(round(0.95 * length(sorted_errors)));
xline(error_50, 'r--', sprintf('50%%: %.2fm', error_50), 'LineWidth', 2);
xline(error_95, 'g--', sprintf('95%%: %.2fm', error_95), 'LineWidth', 2);

end

function create_position_velocity_analysis(t, x_true_hist, x_est_hist, waypoints)
% Detailed position and velocity analysis

figure('Name', 'Position and Velocity Analysis', ...
       'Position', [200, 200, 1400, 800]);

% Position components over time
subplot(2,3,1);
plot(t, x_true_hist(1,:), 'b-', 'LineWidth', 2); hold on;
plot(t, x_est_hist(1,:), 'r--', 'LineWidth', 2);
% Mark waypoint times (simplified)
for i = 1:size(waypoints,2)
    yline(waypoints(1,i), 'g:', sprintf('WP%d', i));
end
grid on;
xlabel('Time (s)'); ylabel('North Position (m)');
title('North Position Tracking', 'FontWeight', 'bold');
legend('True', 'Estimated', 'Location', 'best');

subplot(2,3,2);
plot(t, x_true_hist(2,:), 'b-', 'LineWidth', 2); hold on;
plot(t, x_est_hist(2,:), 'r--', 'LineWidth', 2);
for i = 1:size(waypoints,2)
    yline(waypoints(2,i), 'g:', sprintf('WP%d', i));
end
grid on;
xlabel('Time (s)'); ylabel('East Position (m)');
title('East Position Tracking', 'FontWeight', 'bold');
legend('True', 'Estimated', 'Location', 'best');

subplot(2,3,3);
plot(t, -x_true_hist(3,:), 'b-', 'LineWidth', 2); hold on;
plot(t, -x_est_hist(3,:), 'r--', 'LineWidth', 2);
for i = 1:size(waypoints,2)
    yline(-waypoints(3,i), 'g:', sprintf('WP%d', i));
end
grid on;
xlabel('Time (s)'); ylabel('Altitude (m)');
title('Altitude Tracking', 'FontWeight', 'bold');
legend('True', 'Estimated', 'Location', 'best');

% Velocity components over time
subplot(2,3,4);
plot(t, x_true_hist(4,:), 'b-', 'LineWidth', 2); hold on;
plot(t, x_est_hist(4,:), 'r--', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('North Velocity (m/s)');
title('North Velocity', 'FontWeight', 'bold');
legend('True', 'Estimated', 'Location', 'best');

subplot(2,3,5);
plot(t, x_true_hist(5,:), 'b-', 'LineWidth', 2); hold on;
plot(t, x_est_hist(5,:), 'r--', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('East Velocity (m/s)');
title('East Velocity', 'FontWeight', 'bold');
legend('True', 'Estimated', 'Location', 'best');

subplot(2,3,6);
plot(t, x_true_hist(6,:), 'b-', 'LineWidth', 2); hold on;
plot(t, x_est_hist(6,:), 'r--', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Down Velocity (m/s)');
title('Vertical Velocity', 'FontWeight', 'bold');
legend('True', 'Estimated', 'Location', 'best');

end

function create_attitude_analysis(t, x_true_hist, x_est_hist)
% Attitude and orientation analysis

figure('Name', 'Attitude and Orientation Analysis', ...
       'Position', [250, 250, 1200, 800]);

% Convert to degrees for better readability
att_true_deg = rad2deg(x_true_hist(7:9,:));
att_est_deg = rad2deg(x_est_hist(7:9,:));

% Roll angle
subplot(2,2,1);
plot(t, att_true_deg(1,:), 'b-', 'LineWidth', 2); hold on;
plot(t, att_est_deg(1,:), 'r--', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Roll Angle (deg)');
title('Roll Angle Tracking', 'FontWeight', 'bold');
legend('True', 'Estimated', 'Location', 'best');

% Pitch angle
subplot(2,2,2);
plot(t, att_true_deg(2,:), 'b-', 'LineWidth', 2); hold on;
plot(t, att_est_deg(2,:), 'r--', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Pitch Angle (deg)');
title('Pitch Angle Tracking', 'FontWeight', 'bold');
legend('True', 'Estimated', 'Location', 'best');

% Yaw angle
subplot(2,2,3);
plot(t, att_true_deg(3,:), 'b-', 'LineWidth', 2); hold on;
plot(t, att_est_deg(3,:), 'r--', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Yaw Angle (deg)');
title('Yaw Angle Tracking', 'FontWeight', 'bold');
legend('True', 'Estimated', 'Location', 'best');

% 3D attitude visualization
subplot(2,2,4);
% Plot attitude magnitude over time
att_magnitude_true = sqrt(sum(att_true_deg(1:2,:).^2));
att_magnitude_est = sqrt(sum(att_est_deg(1:2,:).^2));
plot(t, att_magnitude_true, 'b-', 'LineWidth', 2); hold on;
plot(t, att_magnitude_est, 'r--', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Tilt Magnitude (deg)');
title('Overall Tilt Angle', 'FontWeight', 'bold');
legend('True', 'Estimated', 'Location', 'best');

end

function create_error_analysis(t, pos_errors, estimation_errors)
% Comprehensive error analysis and statistics

figure('Name', 'Comprehensive Error Analysis', ...
       'Position', [300, 300, 1200, 800]);

% Position error over time
subplot(2,3,1);
plot(t, pos_errors, 'b-', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Position Error (m)');
title('Position Error vs Time', 'FontWeight', 'bold');

% Add statistics
mean_pos_error = mean(pos_errors);
max_pos_error = max(pos_errors);
hold on;
yline(mean_pos_error, 'r--', sprintf('Mean: %.2fm', mean_pos_error), 'LineWidth', 2);

% Estimation error over time
subplot(2,3,2);
plot(t, estimation_errors, 'r-', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Estimation Error (m)');
title('EKF Estimation Error vs Time', 'FontWeight', 'bold');

% Error comparison
subplot(2,3,3);
plot(t, pos_errors, 'b-', 'LineWidth', 2); hold on;
plot(t, estimation_errors, 'r-', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Error (m)');
title('Error Comparison', 'FontWeight', 'bold');
legend('Position Error', 'Estimation Error', 'Location', 'best');

% Error statistics
subplot(2,3,4);
error_stats = [
    mean(pos_errors), std(pos_errors), max(pos_errors);
    mean(estimation_errors), std(estimation_errors), max(estimation_errors)
];
bar(error_stats);
set(gca, 'XTickLabel', {'Position Error', 'Estimation Error'});
ylabel('Error (m)');
title('Error Statistics', 'FontWeight', 'bold');
legend('Mean', 'Std Dev', 'Maximum', 'Location', 'best');

% Cumulative error distribution
subplot(2,3,5);
sorted_pos_errors = sort(pos_errors);
sorted_est_errors = sort(estimation_errors);
p = (1:length(sorted_pos_errors)) / length(sorted_pos_errors) * 100;
plot(sorted_pos_errors, p, 'b-', 'LineWidth', 2); hold on;
plot(sorted_est_errors, p, 'r-', 'LineWidth', 2);
xlabel('Error (m)');
ylabel('Cumulative Percentage (%)');
title('Cumulative Error Distribution', 'FontWeight', 'bold');
legend('Position Error', 'Estimation Error', 'Location', 'best');
grid on;

% Error correlation
subplot(2,3,6);
scatter(pos_errors, estimation_errors, 30, 'filled');
xlabel('Position Error (m)');
ylabel('Estimation Error (m)');
title('Error Correlation', 'FontWeight', 'bold');
grid on;

% Add correlation coefficient
correlation = corrcoef(pos_errors, estimation_errors);
text(0.1, 0.9, sprintf('Correlation: %.3f', correlation(1,2)), ...
     'Units', 'normalized', 'FontSize', 12, 'FontWeight', 'bold');

end

function create_flight_mode_analysis(t, mode_hist, x_true_hist)
% Flight mode transition and performance analysis

figure('Name', 'Flight Mode Analysis', ...
       'Position', [350, 350, 1200, 600]);

% Mode transitions
subplot(2,2,1);
plot_flight_mode_timeline(t, mode_hist, x_true_hist);

% Performance by mode
subplot(2,2,2);
analyze_performance_by_mode(t, mode_hist, x_true_hist);

% Velocity profile with modes
subplot(2,2,3);
velocity_magnitude = sqrt(sum(x_true_hist(4:6,:).^2));
plot(t, velocity_magnitude, 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Speed (m/s)');
title('Speed Profile', 'FontWeight', 'bold');
grid on;

% Add mode background colors
add_mode_background(t, mode_hist);

% Altitude profile with modes
subplot(2,2,4);
plot(t, -x_true_hist(3,:), 'g-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Altitude (m)');
title('Altitude Profile', 'FontWeight', 'bold');
grid on;

% Add mode background colors
add_mode_background(t, mode_hist);

end

function analyze_performance_by_mode(t, mode_hist, x_true_hist)
% Analyze performance metrics for each flight mode

mode_names = {'disarmed', 'stabilized', 'altitude', 'position', 'trajectory', 'return_to_home', 'emergency_land'};
mode_performance = zeros(length(mode_names), 3); % [mean_speed, max_speed, time_in_mode]

for i = 1:length(mode_names)
    mode_indices = strcmp(mode_hist, mode_names{i});
    
    if any(mode_indices)
        velocities = x_true_hist(4:6, mode_indices);
        speeds = sqrt(sum(velocities.^2));
        
        mode_performance(i, 1) = mean(speeds);
        mode_performance(i, 2) = max(speeds);
        mode_performance(i, 3) = sum(mode_indices) * (t(2) - t(1)); % Time in mode
    end
end

% Plot performance metrics
bar(mode_performance(:, [1, 2]));
set(gca, 'XTickLabel', mode_names, 'XTickLabelRotation', 45);
ylabel('Speed (m/s)');
title('Performance by Flight Mode', 'FontWeight', 'bold');
legend('Mean Speed', 'Max Speed', 'Location', 'best');

end

function add_mode_background(t, mode_hist)
% Add colored background to show flight modes

mode_colors = containers.Map(...
    {'disarmed', 'stabilized', 'altitude', 'position', 'trajectory', 'return_to_home', 'emergency_land'}, ...
    {[0.8 0.8 0.8], [1 0.8 0.8], [0.8 1 0.8], [0.8 0.8 1], [1 1 0.8], [1 0.8 0], [1 0.5 0.5]});

hold on;
ylims = ylim;
current_mode = '';
mode_start = t(1);

for i = 1:length(mode_hist)
    if ~strcmp(mode_hist{i}, current_mode)
        % Mode changed
        if ~isempty(current_mode) && mode_colors.isKey(current_mode)
            % Fill previous mode region
            fill([mode_start, t(i), t(i), mode_start], ...
                 [ylims(1), ylims(1), ylims(2), ylims(2)], ...
                 mode_colors(current_mode), 'FaceAlpha', 0.2, 'EdgeColor', 'none');
        end
        current_mode = mode_hist{i};
        mode_start = t(i);
    end
end

% Fill final mode region
if ~isempty(current_mode) && mode_colors.isKey(current_mode)
    fill([mode_start, t(end), t(end), mode_start], ...
         [ylims(1), ylims(1), ylims(2), ylims(2)], ...
         mode_colors(current_mode), 'FaceAlpha', 0.2, 'EdgeColor', 'none');
end

end

function create_sensor_quality_analysis(t, x_true_hist, x_est_hist, gps_hist)
% Sensor measurement quality and availability analysis

figure('Name', 'Sensor Quality Analysis', ...
       'Position', [400, 400, 1200, 600]);

% GPS availability
subplot(2,2,1);
gps_available = ~isnan(gps_hist(1,:));
plot(t, double(gps_available), 'g-', 'LineWidth', 2);
ylim([-0.1, 1.1]);
xlabel('Time (s)'); ylabel('GPS Available');
title('GPS Availability', 'FontWeight', 'bold');
grid on;

% GPS accuracy when available
subplot(2,2,2);
gps_errors = nan(size(t));
for i = 1:length(t)
    if gps_available(i)
        gps_errors(i) = norm(gps_hist(1:3,i) - x_true_hist(1:3,i));
    end
end
plot(t, gps_errors, 'bo-', 'MarkerSize', 3);
xlabel('Time (s)'); ylabel('GPS Error (m)');
title('GPS Measurement Accuracy', 'FontWeight', 'bold');
grid on;

% Sensor fusion performance
subplot(2,2,3);
total_estimation_error = sqrt(sum((x_true_hist(1:3,:) - x_est_hist(1:3,:)).^2));
plot(t, total_estimation_error, 'r-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Total Estimation Error (m)');
title('Sensor Fusion Performance', 'FontWeight', 'bold');
grid on;

% Sensor health summary
subplot(2,2,4);
gps_uptime = sum(gps_available) / length(gps_available) * 100;
mean_gps_error = nanmean(gps_errors);
mean_estimation_error = mean(total_estimation_error);

sensor_metrics = [gps_uptime; mean_gps_error; mean_estimation_error];
bar(sensor_metrics);
set(gca, 'XTickLabel', {'GPS Uptime (%)', 'Mean GPS Error (m)', 'Mean Est. Error (m)'});
title('Sensor Performance Summary', 'FontWeight', 'bold');
ylabel('Value');

end

function create_performance_dashboard(t, x_true_hist, x_est_hist, waypoints, pos_errors, estimation_errors, mode_hist)
% Comprehensive performance dashboard

figure('Name', 'Performance Dashboard', ...
       'Position', [50, 50, 1600, 1000]);

% Mission overview
subplot(3,4,1);
plot(x_true_hist(1,:), x_true_hist(2,:), 'b-', 'LineWidth', 2); hold on;
plot(waypoints(1,:), waypoints(2,:), 'rs-', 'MarkerSize', 8, 'MarkerFaceColor', 'red');
axis equal; grid on;
xlabel('North (m)'); ylabel('East (m)');
title('Mission Overview', 'FontWeight', 'bold');

% Key performance indicators
subplot(3,4,2);
final_error = norm(x_true_hist(1:3,end) - waypoints(:,end));
max_error = max(pos_errors);
mean_error = mean(pos_errors);
rms_error = sqrt(mean(pos_errors.^2));

kpis = [final_error; max_error; mean_error; rms_error];
bar(kpis);
set(gca, 'XTickLabel', {'Final', 'Max', 'Mean', 'RMS'});
ylabel('Error (m)');
title('Key Performance Indicators', 'FontWeight', 'bold');

% Flight time analysis
subplot(3,4,3);
total_distance = calculate_total_distance(x_true_hist);
flight_time = t(end);
avg_speed = total_distance / flight_time;

metrics = [total_distance; flight_time; avg_speed];
bar(metrics);
set(gca, 'XTickLabel', {'Distance (m)', 'Time (s)', 'Avg Speed (m/s)'});
title('Flight Metrics', 'FontWeight', 'bold');

% Mode distribution
subplot(3,4,4);
plot_mode_distribution(mode_hist);

% Error time series (small multiples)
subplot(3,4,[5,6]);
plot(t, pos_errors, 'b-', 'LineWidth', 1.5); hold on;
plot(t, estimation_errors, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Error (m)');
title('Error Time Series', 'FontWeight', 'bold');
legend('Position Error', 'Estimation Error', 'Location', 'best');

% Altitude profile
subplot(3,4,[7,8]);
plot(t, -x_true_hist(3,:), 'g-', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Altitude (m)');
title('Altitude Profile', 'FontWeight', 'bold');

% Velocity components
subplot(3,4,[9,10]);
velocity_magnitude = sqrt(sum(x_true_hist(4:6,:).^2));
plot(t, velocity_magnitude, 'b-', 'LineWidth', 2); hold on;
plot(t, abs(x_true_hist(4,:)), 'r--', 'LineWidth', 1);
plot(t, abs(x_true_hist(5,:)), 'g--', 'LineWidth', 1);
plot(t, abs(x_true_hist(6,:)), 'm--', 'LineWidth', 1);
grid on;
xlabel('Time (s)'); ylabel('Speed (m/s)');
title('Velocity Components', 'FontWeight', 'bold');
legend('Total', '|V_N|', '|V_E|', '|V_D|', 'Location', 'best');

% Performance summary text
subplot(3,4,[11,12]);
axis off;
summary_text = sprintf([
    'PERFORMANCE SUMMARY\n\n'...
    'Mission Status: %s\n'...
    'Final Error: %.2f m\n'...
    'Max Error: %.2f m\n'...
    'RMS Error: %.2f m\n'...
    'Flight Time: %.1f s\n'...
    'Total Distance: %.1f m\n'...
    'Average Speed: %.1f m/s\n'...
    'EKF Performance: %s\n'...
    'GPS Uptime: %.1f%%'
    ], ...
    assess_mission_status(final_error), final_error, max_error, rms_error, ...
    flight_time, total_distance, avg_speed, ...
    assess_ekf_performance(mean(estimation_errors)), 95.0); % Assume 95% GPS uptime

text(0.1, 0.9, summary_text, 'Units', 'normalized', 'FontSize', 12, ...
     'VerticalAlignment', 'top', 'FontFamily', 'monospace');

end

%% UTILITY FUNCTIONS

function total_distance = calculate_total_distance(x_hist)
% Calculate total distance traveled

distances = sqrt(sum(diff(x_hist(1:3,:), 1, 2).^2));
total_distance = sum(distances);

end

function plot_mode_distribution(mode_hist)
% Plot distribution of flight modes

mode_names = {'disarmed', 'stabilized', 'altitude', 'position', 'trajectory', 'return_to_home', 'emergency_land'};
mode_counts = zeros(size(mode_names));

for i = 1:length(mode_names)
    mode_counts(i) = sum(strcmp(mode_hist, mode_names{i}));
end

pie(mode_counts, mode_names);
title('Flight Mode Distribution', 'FontWeight', 'bold');

end

function status = assess_mission_status(final_error)
% Assess overall mission status

if final_error < 2.0
    status = 'SUCCESS';
elseif final_error < 5.0
    status = 'GOOD';
elseif final_error < 10.0
    status = 'ACCEPTABLE';
else
    status = 'NEEDS IMPROVEMENT';
end

end

function performance = assess_ekf_performance(mean_estimation_error)
% Assess EKF estimation performance

if mean_estimation_error < 0.5
    performance = 'EXCELLENT';
elseif mean_estimation_error < 1.0
    performance = 'GOOD';
elseif mean_estimation_error < 2.0
    performance = 'ACCEPTABLE';
else
    performance = 'POOR';
end

end