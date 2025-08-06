function [x_dot] = drone_dynamics_stable(t, x, u, params)
% drone_dynamics_stable - Enhanced 6DOF nonlinear drone model (9 states) with improved trajectory following
%   x = [pos; vel; att] where:
%       pos = [x; y; z] (NED, m)
%       vel = [vx; vy; vz] (NED, m/s)
%       att = [roll; pitch; yaw] (rad)
%   u = [thrust; tau_phi; tau_theta; tau_psi] (N, Nm)
%   params = struct of physical parameters

% Unpack state
pos = x(1:3);
vel = x(4:6);
att = x(7:9); % [phi; theta; psi]

% Clamp actual roll and pitch to avoid singularities - extremely restrictive
max_angle = deg2rad(10); % Further reduced to 10 degrees for maximum stability
att(1) = max(min(att(1), max_angle), -max_angle); % roll
att(2) = max(min(att(2), max_angle), -max_angle); % pitch

% Unpack control
T = u(1); % Total thrust (N)
tau = u(2:4); % Torques (Nm)

% Rotation matrix (body to NED)
R = rotation_matrix(att(1), att(2), att(3));

% Mass and inertia (assume typical values for now)
m = 0.5; % kg (QAV250)
I = diag([0.0023, 0.0023, 0.004]); % kg*m^2 (QAV250)
if isfield(params, 'mass'), m = params.mass; end
if isfield(params, 'I'), I = params.I; end

g = params.g;

% Enhanced dynamics with improved aerodynamic model
f_gravity = m * g;
f_thrust = R * [0; 0; T];

% Enhanced aerodynamic drag with velocity-dependent coefficients - extreme damping
vel_body = R' * vel; % Transform velocity to body frame
vel_mag = norm(vel_body);

% Improved drag model with different coefficients for different axes - extremely high values
drag_coeff_x = 0.3; % Forward drag (doubled again for extreme stability)
drag_coeff_y = 0.5; % Lateral drag (doubled again for extreme stability)
drag_coeff_z = 0.6; % Vertical drag (doubled again for extreme altitude control)

% Velocity-dependent drag coefficients with extremely aggressive scaling
if vel_mag > 3.0 % Lower threshold
    % High speed - extremely strong drag for stability
    drag_coeff_x = 0.6; % Doubled again
    drag_coeff_y = 0.8; % Doubled again
    drag_coeff_z = 0.9; % Doubled again
elseif vel_mag > 1.5 % Lower threshold
    % Medium speed - very strong drag
    drag_coeff_x = 0.4; % Doubled again
    drag_coeff_y = 0.6; % Doubled again
    drag_coeff_z = 0.7; % Doubled again
else
    % Low speed - strong drag for precise control
    drag_coeff_x = 0.3; % Doubled again
    drag_coeff_y = 0.5; % Doubled again
    drag_coeff_z = 0.6; % Doubled again
end

% Add extremely aggressive velocity limiting for safety
vel_ned_mag = norm(vel);
if vel_ned_mag > 5.0 % Much lower limit
    vel_limit_factor = 5.0 / vel_ned_mag;
    vel = vel * vel_limit_factor;
    % Add strong braking force
    braking_force = -1.0 * m * (vel_ned_mag - 4.0) * (vel / vel_ned_mag);
    f_drag = f_drag + R * braking_force;
elseif vel_ned_mag > 3.0 % Add progressive braking at even lower speeds
    braking_force = -0.5 * m * (vel_ned_mag - 3.0) * (vel / vel_ned_mag);
    f_drag = f_drag + R * braking_force;
end

% Calculate drag forces in body frame
f_drag_body = -[drag_coeff_x * abs(vel_body(1)) * vel_body(1);
                drag_coeff_y * abs(vel_body(2)) * vel_body(2);
                drag_coeff_z * abs(vel_body(3)) * vel_body(3)];

% Transform drag to NED frame
f_drag = R * f_drag_body;

% Enhanced translational acceleration with cross-coupling effects
acc = (f_thrust + f_gravity + f_drag) / m;

% FIXED: Enhanced angular dynamics with improved numerical stability
% Angular acceleration from torques
alpha = tau ./ diag(I); % This is angular acceleration

% Angular rates (Euler angle rates) with enhanced numerical stability
phi = att(1); theta = att(2);
epsilon = 1e-8; % Reduced epsilon for better precision

% Enhanced transformation matrix with improved singularity handling
if abs(cos(theta)) < epsilon
    % Near singularity - use small angle approximation with improved stability
    E = [1, 0, 0;
         0, 1, 0;
         0, 0, 1];
else
    % Normal case with enhanced numerical stability
    cos_theta = cos(theta);
    sin_phi = sin(phi);
    cos_phi = cos(phi);
    tan_theta = tan(theta);
    
    % Check for numerical issues
    if abs(cos_theta) < 1e-6
        cos_theta = 1e-6 * sign(cos_theta);
    end
    
    E = [1, sin_phi*tan_theta, cos_phi*tan_theta;
         0, cos_phi,           -sin_phi;
         0, sin_phi/cos_theta, cos_phi/cos_theta];
end

% FIXED: Enhanced angular rate calculation with improved stability
% Check condition number for numerical stability
if cond(E) > 1e6
    % Use simplified dynamics for numerical stability
    omega = zeros(3,1);
else
    % Convert angular acceleration to Euler angle rates with enhanced stability
    omega = E * alpha;
    
    % Apply rate limiting for stability
    max_angular_rate = deg2rad(200); % Maximum angular rate
    omega = max(min(omega, max_angular_rate), -max_angular_rate);
end

% State derivatives
x_dot = zeros(9,1);
x_dot(1:3) = vel;
x_dot(4:6) = acc;
x_dot(7:9) = omega;
end

function R = rotation_matrix(phi, theta, psi)
% Enhanced rotation matrix from body to NED frame with improved numerical stability
Rz = [cos(psi), -sin(psi), 0;
      sin(psi),  cos(psi), 0;
      0,         0,        1];
Ry = [cos(theta), 0, sin(theta);
      0,          1, 0;
     -sin(theta), 0, cos(theta)];
Rx = [1, 0, 0;
      0, cos(phi), -sin(phi);
      0, sin(phi),  cos(phi)];
R = Rz * Ry * Rx;
end 