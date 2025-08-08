function [x_dot] = drone_dynamics_imu(t, x, imu, params)
% drone_dynamics_imu - 6DOF mechanization for EKF prediction using only IMU data
%   x = [pos; vel; att] where:
%       pos = [x; y; z] (NED, m)
%       vel = [vx; vy; vz] (NED, m/s)
%       att = [roll; pitch; yaw] (rad)
%   imu = [accel_meas; gyro_meas] (accel in m/s^2, gyro in rad/s, both in body frame)
%   params = struct of physical parameters

% Unpack state
pos = x(1:3);
vel = x(4:6);
att = x(7:9); % [phi; theta; psi]

% Unpack IMU
accel_meas = imu(1:3); % body frame
gyro_meas = imu(4:6);  % body frame

g = params.g;

% 1. Update attitude using gyro (Euler integration) with singularity guard
phi = att(1); theta = att(2); psi = att(3);

% Guard cos(theta)
cos_theta = cos(theta);
if abs(cos_theta) < 1e-6
    cos_theta = 1e-6 * sign(cos_theta + (cos_theta==0));
end

sin_phi = sin(phi); cos_phi = cos(phi);
tan_theta = sin(theta)/cos_theta;

E = [1, sin_phi*tan_theta,  cos_phi*tan_theta;
     0, cos_phi,            -sin_phi;
     0, sin_phi/cos_theta,   cos_phi/cos_theta];

att_dot = E * gyro_meas;

% Optional rate limiting for numerical stability
if isfield(params, 'max_angular_rate')
    max_rate = params.max_angular_rate;
else
    max_rate = deg2rad(200);
end
att_dot = max(min(att_dot, max_rate), -max_rate);

% 2. Update velocity using accel (rotate to NED, add gravity) with stable R
R = rotation_matrix(phi, theta, psi);
vel_dot = R * accel_meas + g;

% 3. Position derivative
pos_dot = vel;

% 4. State derivative
x_dot = zeros(9,1);
x_dot(1:3) = pos_dot;
x_dot(4:6) = vel_dot;
x_dot(7:9) = att_dot;
end 