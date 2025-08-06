function animate_drone(t, x_true_hist, x_est_hist, gps_hist, waypoints)
% animate_drone - Animate 3D drone path: true, EKF, and GPS, with drone body
%   t: time vector
%   x_true_hist: [9 x N] true state history (inertial frame)
%   x_est_hist:  [9 x N] EKF estimate history (inertial frame)
%   gps_hist:    [3 x N] GPS measurement history (inertial frame, NaN if not sampled)
%   waypoints:   [3 x num_wp] waypoint positions (inertial frame)

N = length(t);

figure;
h = plot3(0,0,0,'b','LineWidth',2); hold on;
h2 = plot3(0,0,0,'r--','LineWidth',2);
h3 = plot3(0,0,0,'ko','MarkerSize',4,'MarkerFaceColor','y');
h4 = plot3(waypoints(1,:), waypoints(2,:), waypoints(3,:), 'ms--', 'LineWidth', 1.5, 'MarkerSize', 8);
% Only create legend for persistent handles
legend([h h2 h3 h4], {'True','EKF','GPS','Waypoints'});
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Drone 3D Path in Inertial Frame');

% Add ground grid and reference axes
% Robust max_range calculation
all_vals = [abs(waypoints(:)); abs(x_true_hist(:)); abs(x_est_hist(:))];
all_vals = all_vals(isfinite(all_vals)); % Remove NaN/Inf
if isempty(all_vals)
    max_range = 100; % fallback default
else
    max_range = min(max(all_vals), 200); % limit to 200 meters for safety
end

% Warn if simulation data is bad
if any(~isfinite(x_true_hist(:))) || any(~isfinite(x_est_hist(:)))
    warning('Simulation data contains NaN or Inf values. Check your controller and dynamics for instability.');
end

[Xg, Yg] = meshgrid(-20:20:max_range+20, -20:20:max_range+20);
Zg = zeros(size(Xg));
mesh(Xg, Yg, Zg, 'EdgeColor', [0.7 0.7 0.7], 'FaceAlpha', 0.1, 'EdgeAlpha', 0.3, 'DisplayName', 'Ground');
quiver3(0,0,0,20,0,0,'r','LineWidth',2,'MaxHeadSize',2,'DisplayName','X_{inertial}');
quiver3(0,0,0,0,20,0,'g','LineWidth',2,'MaxHeadSize',2,'DisplayName','Y_{inertial}');
quiver3(0,0,0,0,0,20,'b','LineWidth',2,'MaxHeadSize',2,'DisplayName','Z_{inertial}');

% Prism dimensions (meters)
L = 2; W = 1; H = 0.5;

% Create drone body patch handle (not in legend)
drbody = patch('Faces',[], 'Vertices',[], 'FaceColor',[0.2 0.7 1],'FaceAlpha',0.7,'EdgeColor','k','Tag','dronebody');

frame_step = max(1, floor(N/500)); % Show up to 500 frames
for k = 2:frame_step:N
    if ~isvalid(h) || ~isvalid(h2) || ~isvalid(h3) || ~isvalid(drbody)
        break; % Stop if figure or handles are deleted
    end
    set(h, 'XData', x_true_hist(1,1:k), 'YData', x_true_hist(2,1:k), 'ZData', x_true_hist(3,1:k));
    set(h2, 'XData', x_est_hist(1,1:k), 'YData', x_est_hist(2,1:k), 'ZData', x_est_hist(3,1:k));
    set(h3, 'XData', gps_hist(1,1:k), 'YData', gps_hist(2,1:k), 'ZData', gps_hist(3,1:k));
    % Update drone body (rectangular prism)
    pos = x_true_hist(1:3,k); % Inertial frame position
    att = x_true_hist(7:9,k); % Orientation
    [verts, faces] = get_drone_prism(pos, att, L, W, H);
    set(drbody, 'Vertices', verts, 'Faces', faces);
    view(45, 30); % Fixed view for clarity
    drawnow;
    pause(0.02);
end

% Show final traces
set(h, 'XData', x_true_hist(1,:), 'YData', x_true_hist(2,:), 'ZData', x_true_hist(3,:));
set(h2, 'XData', x_est_hist(1,:), 'YData', x_est_hist(2,:), 'ZData', x_est_hist(3,:));
set(h3, 'XData', gps_hist(1,:), 'YData', gps_hist(2,:), 'ZData', gps_hist(3,:));

% Final drone body
pos = x_true_hist(1:3,end);
att = x_true_hist(7:9,end);
[verts, faces] = get_drone_prism(pos, att, L, W, H);
set(drbody, 'Vertices', verts, 'Faces', faces);
end

function [verts, faces] = get_drone_prism(pos, att, L, W, H)
% Get vertices and faces for a rectangular prism at pos, oriented by att (roll, pitch, yaw)
verts0 = [ -L/2 -W/2 -H/2;
            L/2 -W/2 -H/2;
            L/2  W/2 -H/2;
           -L/2  W/2 -H/2;
           -L/2 -W/2  H/2;
            L/2 -W/2  H/2;
            L/2  W/2  H/2;
           -L/2  W/2  H/2];
R = eul2rotm(att');
verts = (R * verts0')' + pos';
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
end 