function plot_results(t, x_true_hist, x_est_hist)
% plot_results - Plot true vs estimated states and errors
%   t: time vector
%   x_true_hist: [9 x N] true state history
%   x_est_hist:  [9 x N] EKF estimate history

labels = {'x (m)','y (m)','z (m)','vx (m/s)','vy (m/s)','vz (m/s)','roll (rad)','pitch (rad)','yaw (rad)'};

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
sgtitle('True vs EKF Estimated States');

figure;
for i = 1:9
    subplot(3,3,i);
    plot(t, x_true_hist(i,:) - x_est_hist(i,:), 'k', 'LineWidth', 1.2);
    ylabel(['Error: ' labels{i}]);
    if i > 6
        xlabel('Time (s)');
    end
    grid on;
end
sgtitle('Estimation Errors');
end 