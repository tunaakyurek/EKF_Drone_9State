function plot_results_improved(t, x_true_hist, x_est_hist, waypoints)
% plot_results_improved - Enhanced plotting with turn analysis
%   t: time vector
%   x_true_hist: [9 x N] true state history
%   x_est_hist:  [9 x N] EKF estimate history
%   waypoints:   [3 x num_wp] waypoint positions

labels = {'x (m)','y (m)','z (m)','vx (m/s)','vy (m/s)','vz (m/s)','roll (rad)','pitch (rad)','yaw (rad)'};

% Standard state comparison plots
figure;
for i = 1:9
    subplot(3,3,i);
    plot(t, x_true_hist(i,:), 'b', 'LineWidth', 1.2); hold on;
    plot(t, x_est_hist(i,:), 'r--', 'LineWidth', 1.2);
    ylabel(labels{i});
    if i > 6
        xlabel('Time (s)');
    end
    legend('True','EKF');
    grid on;
end
sgtitle('True vs EKF Estimated States (Improved)');

% Error analysis with turn detection
figure;
for i = 1:9
    subplot(3,3,i);
    error = x_true_hist(i,:) - x_est_hist(i,:);
    plot(t, error, 'k', 'LineWidth', 1.2);
    ylabel(['Error: ' labels{i}]);
    if i > 6
        xlabel('Time (s)');
    end
    grid on;
    
    % Add turn indicators
    hold on;
    % Mark waypoint transitions (potential turn points)
    for wp = 1:size(waypoints,2)
        if wp < size(waypoints,2)
            % Estimate turn time based on waypoint distance
            turn_time = wp * 20; % Approximate turn time
            if turn_time <= t(end)
                xline(turn_time, 'r--', 'LineWidth', 1);
            end
        end
    end
end
sgtitle('Estimation Errors with Turn Indicators');

% Position error magnitude over time
figure;
pos_error = sqrt(sum((x_true_hist(1:3,:) - x_est_hist(1:3,:)).^2, 1));
plot(t, pos_error, 'b', 'LineWidth', 2);
ylabel('Position Error Magnitude (m)');
xlabel('Time (s)');
title('Position Estimation Error Over Time');
grid on;

% Add turn indicators
hold on;
for wp = 1:size(waypoints,2)
    if wp < size(waypoints,2)
        turn_time = wp * 20;
        if turn_time <= t(end)
            xline(turn_time, 'r--', 'LineWidth', 1);
            text(turn_time, max(pos_error), sprintf('WP%d', wp), 'Rotation', 90);
        end
    end
end

% Velocity and attitude error analysis
figure;
subplot(2,1,1);
vel_error = sqrt(sum((x_true_hist(4:6,:) - x_est_hist(4:6,:)).^2, 1));
plot(t, vel_error, 'g', 'LineWidth', 2);
ylabel('Velocity Error Magnitude (m/s)');
title('Velocity Estimation Error Over Time');
grid on;

subplot(2,1,2);
att_error = sqrt(sum((x_true_hist(7:9,:) - x_est_hist(7:9,:)).^2, 1));
plot(t, att_error, 'm', 'LineWidth', 2);
ylabel('Attitude Error Magnitude (rad)');
xlabel('Time (s)');
title('Attitude Estimation Error Over Time');
grid on;

% Add turn indicators
for wp = 1:size(waypoints,2)
    if wp < size(waypoints,2)
        turn_time = wp * 20;
        if turn_time <= t(end)
            subplot(2,1,1);
            xline(turn_time, 'r--', 'LineWidth', 1);
            subplot(2,1,2);
            xline(turn_time, 'r--', 'LineWidth', 1);
        end
    end
end

% 3D trajectory comparison
figure;
plot3(x_true_hist(1,:), x_true_hist(2,:), x_true_hist(3,:), 'b', 'LineWidth', 2);
hold on;
plot3(x_est_hist(1,:), x_est_hist(2,:), x_est_hist(3,:), 'r--', 'LineWidth', 2);
plot3(waypoints(1,:), waypoints(2,:), waypoints(3,:), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'y');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Trajectory Comparison');
legend('True Path', 'EKF Estimate', 'Waypoints');
grid on;

% Statistical analysis
fprintf('\n=== EKF Performance Analysis ===\n');
fprintf('Mean Position Error: %.3f m\n', mean(pos_error));
fprintf('Max Position Error: %.3f m\n', max(pos_error));
fprintf('Mean Velocity Error: %.3f m/s\n', mean(vel_error));
fprintf('Max Velocity Error: %.3f m/s\n', max(vel_error));
fprintf('Mean Attitude Error: %.3f rad\n', mean(att_error));
fprintf('Max Attitude Error: %.3f rad\n', max(att_error));

% Turn-specific analysis
turn_times = [20, 40, 60, 80, 100]; % Approximate turn times
for i = 1:length(turn_times)
    if turn_times(i) <= t(end)
        [~, idx] = min(abs(t - turn_times(i)));
        fprintf('Turn %d (%.1fs): Pos Error = %.3f m, Vel Error = %.3f m/s\n', ...
            i, turn_times(i), pos_error(idx), vel_error(idx));
    end
end
end 