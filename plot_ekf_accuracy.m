function plot_ekf_accuracy(t, x_true_hist, x_est_hist)
% plot_ekf_accuracy  Quick post-flight EKF performance figures
%
%   Generates three subplots:
%     1) Position error vs time (XYZ)
%     2) Velocity error vs time (XYZ)
%     3) Attitude error vs time (roll-pitch-yaw in deg)
%
%   Inputs
%     t            – time vector [1×N]
%     x_true_hist  – true states  [9×N]
%     x_est_hist   – estimated    [9×N]

pos_err = x_true_hist(1:3,:) - x_est_hist(1:3,:);
vel_err = x_true_hist(4:6,:) - x_est_hist(4:6,:);
att_err = wrapToPi(x_true_hist(7:9,:) - x_est_hist(7:9,:));

figure('Name','EKF Accuracy','Position',[100 100 1200 800]);
subplot(3,1,1);
plot(t, pos_err'); grid on;
legend({'x','y','z'}); ylabel('Position error (m)');

title('EKF Position Error');
subplot(3,1,2);
plot(t, vel_err'); grid on;
legend({'v_x','v_y','v_z'}); ylabel('Velocity error (m/s)');

title('EKF Velocity Error');
subplot(3,1,3);
plot(t, rad2deg(att_err)'); grid on;
legend({'roll','pitch','yaw'});
ylabel('Attitude error (deg)'); xlabel('Time (s)');

title('EKF Attitude Error');
end